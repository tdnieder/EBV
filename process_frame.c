/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */
 
/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define  NumFgrCol	2 //blue and red stones
#define BLUE	0
#define RED		1

//------------------------------------------------------------------------------------------
#define YCBCR 	0	//1 = detect in YCbCr colors, 0 detect in RGB colros
//------------------------------------------------------------------------------------------

int regionColor[20];

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

bool ManualThreshold;

/* skip pixel at border */
const int Border = 2;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

/* size of centroid marker */
const int SizeCross = 10;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void ChangeDetectionRGB();
void ChangeDetectionYCbCr();


void ResetProcess()
{
	//called when "reset" button is pressed
	ManualThreshold = true;
}


void ProcessFrame() {
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		ManualThreshold = true;
	} else {
		if(YCBCR == 1)
			ChangeDetectionYCbCr();
		else
			ChangeDetectionRGB();

		DrawBoundingBoxes();
	}
}

void Erode_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											   *(p-1)    & *p      & *(p+1)    &
											   *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											        *(p-1)    | *p      | *(p+1)    |
											        *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}
}


void DetectRegions() {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for(i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[INDEX0][i] = data.u8TempImage[THRESHOLD][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[INDEX0];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);
}


void DrawBoundingBoxes() {
	uint16 o;
	int color;
	for(o = 0; o < ImgRegions.noOfObjects; o++) {
		if(ImgRegions.objects[o].area > MinArea) {
			if(regionColor[o] == BLUE)
				color = 4;
			else
				color = 2;
			DrawBoundingBox(ImgRegions.objects[o].bboxLeft, ImgRegions.objects[o].bboxTop,
							ImgRegions.objects[o].bboxRight, ImgRegions.objects[o].bboxBottom, false, color);

			DrawLine(ImgRegions.objects[o].centroidX-SizeCross, ImgRegions.objects[o].centroidY,
					 ImgRegions.objects[o].centroidX+SizeCross, ImgRegions.objects[o].centroidY, color);
			DrawLine(ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY-SizeCross,
								 ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY+SizeCross, color);

		}
	}
}

void ChangeDetectionRGB() {
	//const int NumFgrCol = 2;	//blue, red backwards BGR
	uint8 FrgCol[NumFgrCol][3] = {{110, 75, 61}, {34, 44, 162}};
	int r, c, frg, p, o;

	memset(data.u8TempImage[INDEX0], 0, IMG_SIZE);
	memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);


	//loop over the rows
	for(r = 0; r < nr*nc; r += nc) {
		//loop over the columns
		for(c = 0; c < nc; c++) {
			//loop over the different Frg colors and find smallest difference
			int MinDif = 1 << 30;
			int MinInd = 0;
			for(frg = 0; frg < NumFgrCol; frg++) {
				int Dif = 0;
				//loop over the color planes (r, g, b) and sum up the difference
				for(p = 0; p < NUM_COLORS; p++) {
					Dif += abs((int) data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+p]-
					(int) FrgCol[frg][p]);
				}
				if(Dif < MinDif) {
					MinDif = Dif;
					MinInd = frg;
				}
			}
			//if the difference is smaller than threshold value
			if(MinDif < data.ipc.state.nThreshold) {
				//set pixel value to 255 in THRESHOLD image for further processing
				//(we use only the first third of the image buffer)
				data.u8TempImage[INDEX1][(r+c)] = 255;
				//set pixel value to Frg color in BACKGROUND image for visualization
				for(p = 0; p < NUM_COLORS; p++) {
					data.u8TempImage[BACKGROUND][(r+c)*NUM_COLORS+p] = FrgCol[MinInd][p];
				}
			}
		}
	}
	Erode_3x3(INDEX1, INDEX0);
	Dilate_3x3(INDEX0, THRESHOLD);
	DetectRegions();
	//loop over objects
	for(o = 0; o < ImgRegions.noOfObjects; o++) {
		if(ImgRegions.objects[o].area > MinArea){
			int hist[2]; //make histogramm for color Blue and Red
			hist[BLUE] = 0;
			hist[RED] = 0;
			//get pointer to root run of current object
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
			//loop over runs of current object
			do {
				//loop over pixel of current run
				for(c = currentRun->startColumn; c <= currentRun->endColumn; c++) {
					int r = currentRun->row;
					//loop over color planes of pixel
					for(p = 1; p < NUM_COLORS; p++) {
						//addressing individual pixel at row r, column c and color p
						if(data.u8TempImage[BACKGROUND][(r*nc+c)*NUM_COLORS+p] == FrgCol[0][p]){
							hist[BLUE]++;
						}
						else
							hist[RED]++;
					}
				}
				currentRun = currentRun->next; //get next run of current object
			} while(currentRun != NULL); //end of current object
			if(hist[BLUE] > hist[RED]){
				regionColor[o] = BLUE;
			}
			else{
				regionColor[o] = RED;
			}
		}
	}
}

void ChangeDetectionYCbCr() {
	//colors blue and red in YCbCr (here CrCbY, backward order)
	uint8 FrgCol[NumFgrCol][3] = {{95, 155, 95}, {90, 110, 180}};
	int r, c, frg, p, o;

	memset(data.u8TempImage[INDEX1], 0, IMG_SIZE);
	memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
	memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);

	for(r = 0; r < nr*nc; r += nc) {
		//loop over the columns
		for(c = 0; c < nc; c++) {
			//get rgb values (order is actually bgr!)
			float B_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+0];
			float G_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+1];
			float R_ = data.u8TempImage[SENSORIMG][(r+c)*NUM_COLORS+2];
			uint8 Y_ = (uint8) ( 0 + 0.299*R_ + 0.587*G_ + 0.114*B_);
			uint8 Cb_ = (uint8) (128 - 0.169*R_ - 0.331*G_ + 0.500*B_);
			uint8 Cr_ = (uint8) (128 + 0.500*R_ - 0.419*G_ - 0.081*B_);
			//we write result to THRESHOLD
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+0] = Y_;
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+1] = Cb_;
			data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+2] = Cr_;
		}
	}

	//loop over the rows
	for(r = 0; r < nr*nc; r += nc) {
		//loop over the columns
		for(c = 0; c < nc; c++) {
			//loop over the different Frg colors and find smallest difference
			int MinDif = 1 << 30;
			int MinInd = 0;
			for(frg = 0; frg < NumFgrCol; frg++) {
				int Dif = 0;
				//loop over the color planes (r, g, b) and sum up the difference
				for(p = 1; p < NUM_COLORS; p++) {
					Dif += abs((int) data.u8TempImage[THRESHOLD][(r+c)*NUM_COLORS+p]-
					(int) FrgCol[frg][p]);
				}
				if(Dif < MinDif) {
					MinDif = Dif;
					MinInd = frg;
				}
			}
			//if the difference is smaller than threshold value
			if(MinDif < data.ipc.state.nThreshold) {
				//set pixel value to 255 in THRESHOLD image for further processing
				//(we use only the first third of the image buffer)
				data.u8TempImage[INDEX1][(r+c)] = 255;
				//set pixel value to Frg color in BACKGROUND image for visualization
				for(p = 1; p < NUM_COLORS; p++) {
					data.u8TempImage[BACKGROUND][(r+c)*NUM_COLORS+p] = FrgCol[MinInd][p];
				}
			}
		}
	}
	Erode_3x3(INDEX1, INDEX0);
	Dilate_3x3(INDEX0, THRESHOLD);
	DetectRegions();
	//loop over objects
	for(o = 0; o < ImgRegions.noOfObjects; o++) {
		if(ImgRegions.objects[o].area > MinArea){
			int hist[2]; //make histogramm for color Blue and Red
			hist[BLUE] = 0;
			hist[RED] = 0;
			//get pointer to root run of current object
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;
			//loop over runs of current object
			do {
				//loop over pixel of current run
				for(c = currentRun->startColumn; c <= currentRun->endColumn; c++) {
					int r = currentRun->row;
					//loop over color planes of pixel
					for(p = 1; p < NUM_COLORS; p++) {
						//addressing individual pixel at row r, column c and color p
						if(data.u8TempImage[BACKGROUND][(r*nc+c)*NUM_COLORS+p] == FrgCol[0][p]){
							hist[BLUE]++;
						}
						else
							hist[RED]++;
					}
				}
				currentRun = currentRun->next; //get next run of current object
			} while(currentRun != NULL); //end of current object
			if(hist[BLUE] > hist[RED]){
				regionColor[o] = BLUE;
			}
			else{
				regionColor[o] = RED;
			}
		}
	}
}
