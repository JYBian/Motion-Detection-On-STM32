/*****************************************************************************
File name: main.cpp
Description: Used to carry out the three modes on board, part of the functions are based on TA Michael's demo.
Author: Junyu Bian (some codes used from TA Michael's demo)
Date: 12/16/2020
*****************************************************************************/

/* system imports */
#include <mbed.h>
#include <math.h>
#include <USBSerial.h>

/* user imports */
#include "LIS3DSH.h"

/* USBSerial library for serial terminal */
USBSerial serial(0x1f00,0x2012,0x0001,false);

/* LIS3DSH Library for accelerometer  - using SPI*/
LIS3DSH acc(PA_7, SPI_MISO, SPI_SCK, PE_3);

/* LED output */
DigitalOut MyLED6(LED6);					// LED6 - blue - stands for JumpJacks
DigitalOut MyLED4(LED4);					// LED4 - green - stands for Squarts
DigitalOut MyLED3(LED3);					// LED3 - orange - stands for SitUps
DigitalOut MyLED5(LED5);					// LED5 - red - stands for PushUps

/* Button input */
DigitalIn MyButton(BUTTON1);				// User Button to receive user interrupts

/* Final variables */
const int VERY_SHORT_TIME = 200;			// to control fast blinking
const int SHORT_TIME = 500;					// to control blink frequency
const int LONG_TIME = 3000;					// to wait for user input
const int ON = 1;							// ON state of LED and User Button 
const int OFF = 0;							// OFF state of LED and User Button 
const float PI = 3.1415926;					// PI		
const int PRESAMPLE_LENGTH = 20;			// presample length

/* Internal variables */
bool isButtonPressed = false;				// button state
int16_t xAccel = 0;							// acceleration in x (raw)
int16_t yAccel = 0;							// acceleration in y (raw)
int16_t zAccel = 0; 						// acceleration in z (raw)
float g_x = 0;  							// acceleration in x 
float g_y = 0; 								// acceleration in y
float g_z = 0;  							// acceleration in z 
float angleX = 0;							// angle relative to x axis
float angleY = 0;							// angle relative to y axis
float angleZ = 0;							// angle relative to z axis
double presamplesBuffer[PRESAMPLE_LENGTH][3];	// buffer storing preSamples
double tempBuffer[3];						// intermedia buffer

/* Filter Parameters  */
const uint8_t N = 20; 						// filter length
float ringbufX[N];							// sample buffer x
float ringbufY[N];							// sample buffer y
float ringbufZ[N];							// sample buffer z
uint8_t ringbuf_index_x = 0;				// index to insert sample x
uint8_t ringbuf_index_y = 0;				// index to insert sample y
uint8_t ringbuf_index_z = 0;				// index to insert sample z


/*************************************************
Function: sampling
Description: Code modified from TA Michael's demo, used to sample one data on each axis
Calls: None
Called By: sampleTwoSeconds()
Others: store the sampled data in tempBuffer
*************************************************/
void sampling() {
	/* read data from the accelerometer */
	acc.ReadData(&xAccel, &yAccel, &zAccel);

	/* normalise to 1g */
	g_x = (float)xAccel/17694.0;
	g_y = (float)yAccel/17694.0;
	g_z = (float)zAccel/17694.0;

	/* insert in to circular buffer */
	ringbufX[ringbuf_index_x++] = g_x;
	ringbufY[ringbuf_index_y++] = g_y;
	ringbufZ[ringbuf_index_z++] = g_z;

	/* at the end of the buffer, wrap around to the beginning */
	if (ringbuf_index_x >= N) {
		ringbuf_index_x = 0;
	}
	if (ringbuf_index_y >= N) {
		ringbuf_index_y = 0;
	}
	if (ringbuf_index_z >= N) {
		ringbuf_index_z = 0;
	}

	/********** START of filtering ********************/
	float g_x_filt = 0;
	float g_y_filt = 0;
	float g_z_filt = 0;

	/* add all samples */
	for (uint8_t i = 0; i < N; i++) {
		g_x_filt += ringbufX[i];
	}
	for (uint8_t i = 0; i < N; i++) {
		g_y_filt += ringbufY[i];
	}
	for (uint8_t i = 0; i < N; i++) {
		g_z_filt += ringbufZ[i];
	}

	/* divide by number of samples */
	g_x_filt /= (float)N;
	g_y_filt /= (float)N;
	g_z_filt /= (float)N;
	/* restrict to 1g (acceleration not decoupled from orientation) */
	if (g_x_filt > 1) {
		g_x_filt = 1;
	}
	if (g_y_filt > 1) {
		g_y_filt = 1;
	}
	if (g_z_filt > 1) {
		g_z_filt = 1;
	}
	/********** END of filtering **********************/

	/* compute angle in degrees */
	angleX = 180*acos(g_x_filt)/PI;
	angleY = 180*acos(g_y_filt)/PI;
	angleZ = 180*acos(g_z_filt)/PI;

	/* Storing the sampling result */
	tempBuffer[0] = angleX;
	tempBuffer[1] = angleY;
	tempBuffer[2] = angleZ;
}


/*************************************************
Function: sampleTwoSeconds
Description: sampling for two seconds
Calls: sampling()
Called By: routinedExercise(), freeToExercise()
Others: store the sampled data in presamplesBuffer
*************************************************/
void sampleTwoSeconds() {
    /* one sampel per 0.1s, so for two seconds, we need 20 times, which is PRESAMPLE_LENGTH */
    for (int i = 0; i < PRESAMPLE_LENGTH; i++) {
		sampling();
        presamplesBuffer[i][0] = tempBuffer[0];
		presamplesBuffer[i][1] = tempBuffer[1];
		presamplesBuffer[i][2] = tempBuffer[2];
		wait_ms(100);
    }
}


/*************************************************
Function: isSU
Description: make judgement whether the exercise is SitUps based on presample data
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: returns a bool value indicating whether SitUps or not
*************************************************/
bool isSU() {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	/* calculating the average angle value for each axis */
    for (int i = 0; i < PRESAMPLE_LENGTH; i++) {
		sumX += presamplesBuffer[i][0];
		sumY += presamplesBuffer[i][1];
		sumZ += presamplesBuffer[i][2];
	}

	if (sumZ/PRESAMPLE_LENGTH <= 180 && sumZ/PRESAMPLE_LENGTH >= 120 
		&& sumY/PRESAMPLE_LENGTH <= 140 && sumY/PRESAMPLE_LENGTH >= 80
		&& sumX/PRESAMPLE_LENGTH <= 100 && sumX/PRESAMPLE_LENGTH >= 60  ) {
		MyLED3 = ON;
		serial.printf("SitUps\n");
		return true;
	}

	return false;
}


/*************************************************
Function: countSU
Description: if isSU returns true, start counting using this function
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: can be interrupted by user button to display process, count up to 5 reputations and stop
*************************************************/
void countSU(int initVal) {
    int countSU = initVal;

	while (countSU < 5) {
		/* when button not triggered and 5 reputations not finished, continue detecting */
		while (MyButton != ON && countSU < 5) {
			sampleTwoSeconds();
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when maximum deteted, regards as one reputation finished */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					countSU++;
				}
			}
		}

		/* when button pressed, display the process and continue counting */
		for (int i = 0; i < countSU; i++) {
			MyLED3 = ON;
			thread_sleep_for(SHORT_TIME);
			MyLED3 = OFF;
			thread_sleep_for(SHORT_TIME);
		}
	}

}


/*************************************************
Function: isJJ
Description: make judgement whether the exercise is JumpJacks based on presample data
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: returns a bool value indicating whether JumpJacks or not
*************************************************/
bool isJJ() {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	/* calculating the average angle value for each axis */
    for (int i = 0; i < PRESAMPLE_LENGTH; i++) {
		sumX += presamplesBuffer[i][0];
		sumY += presamplesBuffer[i][1];
		sumZ += presamplesBuffer[i][2];
	}
	if (sumZ/PRESAMPLE_LENGTH <= 80 && sumZ/PRESAMPLE_LENGTH >= 30 
		&& sumY/PRESAMPLE_LENGTH <= 120 && sumY/PRESAMPLE_LENGTH >= 80
		&& sumX/PRESAMPLE_LENGTH <= 160 && sumX/PRESAMPLE_LENGTH >= 60  ) {
		MyLED6 = ON;
		serial.printf("JumpJacks\n");
		return true;
	}
	return false;
}


/*************************************************
Function: countJJ
Description: if isJJ returns true, start counting using this function
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: can be interrupted by user button to display process, count up to 5 reputations and stop
*************************************************/
void countJJ(int initVal) {
    int countJJ = initVal;

	while (countJJ < 5) {
		while (MyButton != ON && countJJ < 5) {
			sampleTwoSeconds();
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when button not triggered and 5 reputations not finished, continue detecting */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					countJJ++;
				}
			}
		}

		/* when button pressed, display the process and continue counting */
		for (int i = 0; i < countJJ; i++) {
			MyLED6 = ON;
			thread_sleep_for(SHORT_TIME);
			MyLED6 = OFF;
			thread_sleep_for(SHORT_TIME);
		}
	}

}


/*************************************************
Function: isPU
Description: make judgement whether the exercise is PushUps based on presample data
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: returns a bool value indicating whether PushUps or not
*************************************************/
bool isPU() {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	/* calculating the average angle value for each axis */
    for (int i = 0; i < PRESAMPLE_LENGTH; i++) {
		sumX += presamplesBuffer[i][0];
		sumY += presamplesBuffer[i][1];
		sumZ += presamplesBuffer[i][2];
	}
	if (sumZ/PRESAMPLE_LENGTH <= 40 && sumZ/PRESAMPLE_LENGTH >= 20 
		&& sumY/PRESAMPLE_LENGTH <= 90 && sumY/PRESAMPLE_LENGTH >= 60
		&& sumX/PRESAMPLE_LENGTH <= 100 && sumX/PRESAMPLE_LENGTH >= 80  ) {
		MyLED5 = ON;
		serial.printf("PushUps\n");
		return true;
	}
	return false;
}


/*************************************************
Function: countPU
Description: if isPU returns true, start counting using this function
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: can be interrupted by user button to display process, count up to 5 reputations and stop
*************************************************/
void countPU(int initVal) {
    int countPU = initVal;

	while (countPU < 5) {
		while (MyButton != ON && countPU < 5) {
			sampleTwoSeconds();
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when button not triggered and 5 reputations not finished, continue detecting */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					countPU++;
				}
			}
		}

		/* when button pressed, display the process and continue counting */
		for (int i = 0; i < countPU; i++) {
			MyLED5 = ON;
			thread_sleep_for(SHORT_TIME);
			MyLED5 = OFF;
			thread_sleep_for(SHORT_TIME);
		}
	}
}


/*************************************************
Function: isS
Description: make judgement whether the exercise is Squart based on presample data
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: returns a bool value indicating whether Squart or not
*************************************************/
bool isS() {
	double sumX = 0;
	double sumY = 0;
	double sumZ = 0;

	/* calculating the average angle value for each axis */
    for (int i = 0; i < PRESAMPLE_LENGTH; i++) {
		sumX += presamplesBuffer[i][0];
		sumY += presamplesBuffer[i][1];
		sumZ += presamplesBuffer[i][2];
	}
	if (sumZ/PRESAMPLE_LENGTH <= 180 && sumZ/PRESAMPLE_LENGTH >= 120 
		&& sumY/PRESAMPLE_LENGTH <= 140 && sumY/PRESAMPLE_LENGTH >= 80
		&& sumX/PRESAMPLE_LENGTH <= 100 && sumX/PRESAMPLE_LENGTH >= 60  ) {
		MyLED4 = ON;
		serial.printf("Squarts\n");
		return true;
	}
	return false;
}


/*************************************************
Function: countS
Description: if isS returns true, start counting using this function
Calls: None
Called By: routinedExercise(), freeToExercise()
Others: can be interrupted by user button to display process, count up to 5 reputations and stop
*************************************************/
void countS(int initVal) {
    int countS = initVal;

	while (countS < 5) {
		while (MyButton != ON && countS < 5) {
			sampleTwoSeconds();
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when button not triggered and 5 reputations not finished, continue detecting */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					countS++;
				}
			}
		}

		/* when button pressed, display the process and continue counting */
		for (int i = 0; i < countS; i++) {
			MyLED4 = ON;
			thread_sleep_for(SHORT_TIME);
			MyLED4 = OFF;
			thread_sleep_for(SHORT_TIME);
		}
	}
}


/*************************************************
Function: waitingLight
Description: indicating waiting state
Calls: None
Called By: main(), freeToExercise()
Others: 

Waiting interface, 
used when no user button input detected,

LED will blink in led3 -> led5 -> led6 -> led4 order, 
to proceed, user button interrupt is needed,
*************************************************/
void waitingLight() {
	while(MyButton != ON) {
		MyLED3 = ON;
		thread_sleep_for(SHORT_TIME);
		MyLED3 = OFF;
		MyLED5 = ON;
		thread_sleep_for(SHORT_TIME);
		MyLED5 = OFF;
		MyLED6 = ON;
		thread_sleep_for(SHORT_TIME);
		MyLED6 = OFF;
		MyLED4 = ON;
		thread_sleep_for(SHORT_TIME);
		MyLED4 = OFF;
		serial.printf("Waiting\r\n");
	}
	return;
}


/*************************************************
Function: freeToExercise
Description: free exercise model
Calls: None
Called By: main()
Others: 

This is the model when user can do any exercise as wished,
when entered, four LEDs will be blinking in circle quickly,
type of exercise will be detected automatically,

led3 indicates Situps,
led5 indicates Pushups,
led6 indicates Jumping Jacks,
led4 indicate Squats,

for each kind of exercise, the program will count for 5 times,
each LED will blink corresponding times to indicate how many repetitions has been counted,
after finished, all the four lights will be blinking.

User can choose to push user button while blinking to move into next model,
otherwise, user is needed to do one of the exercise above.
*************************************************/
void freeToExercise() {
	/* blinking LEDs */
	for (int i = 0; i < 3; i++) {
		MyLED3 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED3 = OFF;
		MyLED5 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED5 = OFF;
		MyLED6 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED6 = OFF;
		MyLED4 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED4 = OFF;
	}

	while(!isButtonPressed) {
		/* get the state of user button */
		isButtonPressed = MyButton;

		/* if user press button instead of doing exercise, return directly */
		if (isButtonPressed) {
			return;
		}

		/* presampling 2 secs for exercise detection */
		sampleTwoSeconds();

		if (isSU()) {
			int initVal = 0;
			/* check data presampled in the previous 2secs */
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when maximum deteted, regards as one reputation finished */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					initVal++;
				}
			}
			countSU(initVal);
			for(int i = 0; i < 3; i++) {
				MyLED3 = ON;
				MyLED5 = ON;
				MyLED6 = ON;
				MyLED4 = ON;
				thread_sleep_for(SHORT_TIME);
				MyLED3 = OFF;
				MyLED5 = OFF;
				MyLED6 = OFF;
				MyLED4 = OFF;
				thread_sleep_for(SHORT_TIME);
			}
			return;
		}

		if (isPU()) {
			int initVal = 0;
			/* check data presampled in the previous 2secs */
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when maximum deteted, regards as one reputation finished */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					initVal++;
				}
			}
			countPU(initVal);
			for(int i = 0; i < 3; i++) {
				MyLED3 = ON;
				MyLED5 = ON;
				MyLED6 = ON;
				MyLED4 = ON;
				thread_sleep_for(SHORT_TIME);
				MyLED3 = OFF;
				MyLED5 = OFF;
				MyLED6 = OFF;
				MyLED4 = OFF;
				thread_sleep_for(SHORT_TIME);
			}
			return;
		}

		if (isJJ()) {
			int initVal = 0;
			/* check data presampled in the previous 2secs */
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when maximum deteted, regards as one reputation finished */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					initVal++;
				}
			}
			countJJ(initVal);
			for(int i = 0; i < 3; i++) {
				MyLED3 = ON;
				MyLED5 = ON;
				MyLED6 = ON;
				MyLED4 = ON;
				thread_sleep_for(SHORT_TIME);
				MyLED3 = OFF;
				MyLED5 = OFF;
				MyLED6 = OFF;
				MyLED4 = OFF;
				thread_sleep_for(SHORT_TIME);
			}
			return;
		}

		if (isS()) {
			int initVal = 0;
			/* check data presampled in the previous 2secs */
			for (int i = 1; i < PRESAMPLE_LENGTH-1; i++) {
				/* when maximum deteted, regards as one reputation finished */
				if (presamplesBuffer[i][1] > presamplesBuffer[i-1][1] 
					&& presamplesBuffer[i][1] > presamplesBuffer[i+1][1]) {
					initVal++;
				}
			}
			countS(initVal);
			for(int i = 0; i < 3; i++) {
				MyLED3 = ON;
				MyLED5 = ON;
				MyLED6 = ON;
				MyLED4 = ON;
				thread_sleep_for(SHORT_TIME);
				MyLED3 = OFF;
				MyLED5 = OFF;
				MyLED6 = OFF;
				MyLED4 = OFF;
				thread_sleep_for(SHORT_TIME);
			}
			return;
		}
	}
	return;
}


/*************************************************
Function: routinedExercise
Description: routined exercise model
Calls: None
Called By: main()
Others: 

This is the mode used when user wish to do routined exercise,
when entered, the four LEDs will be blinking in reverse order quickly,
type of exercise has been arranged as follows:

1. 1st gourp: 5 Situps
2. 2nd group: 5 Pushups
3. 3rd group: 5 Jumping Jacks
4. 4rd group: 5 Squats

each LED will blink corresponding times to indicate how many repetitions has been counted,
when one group finished, all the four lights will be circuling waiting for user to continue next group,
after finishing all four groups, the four LEDs will be blinking together,
user should push user button to restart the program from beginning.
*************************************************/
void routinedExercise() {
	for (int i = 0; i < 3; i++) {
		MyLED4 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED4 = OFF;
		MyLED6 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED6 = OFF;
		MyLED5 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED5 = OFF;
		MyLED3 = ON;
		thread_sleep_for(VERY_SHORT_TIME);
		MyLED3 = OFF;
	}

	serial.printf("Rountined Model\r\n");

	bool sitUpsFinished = false;
	bool pushUpsFinished = false;
	bool jumpJacksFinished = false;
	bool squartsFinished = false;

	while(true) {
		/* get the state of user button */
		isButtonPressed = MyButton;

		/* detection for Situps */
		if (isButtonPressed 
			&& !sitUpsFinished 
			&& !pushUpsFinished 
			&& !jumpJacksFinished 
			&& !squartsFinished) {
			/* LED3 turns on, reminding user to do Situps. */
			MyLED3 = ON;
			/* different with free mode, no presampling process, initVal is 0 */
			countSU(0);
			sitUpsFinished = true;
		}
		/* get the state of user button */
		isButtonPressed = MyButton;

		/* detection for Pushups */
		if (isButtonPressed 
			&& sitUpsFinished 
			&& !pushUpsFinished
			&& !jumpJacksFinished 
			&& !squartsFinished) {
			/* LED5 turns on, reminding user to do Pushups. */
			MyLED5 = ON;
			/* different with free mode, no presampling process, initVal is 0 */
			countPU(0);
			pushUpsFinished = true;
		}
		/* get the state of user button */
		isButtonPressed = MyButton;
		
		/* detection for Jumping Jacks */
		if (isButtonPressed 
			&& sitUpsFinished 
			&& pushUpsFinished 
			&& !jumpJacksFinished 
			&& !squartsFinished) {
			/* LED6 turns on, reminding user to do JumpingJacks. */
			MyLED6 = ON;
			/* different with free mode, no presampling process, initVal is 0 */
			countJJ(0);
			jumpJacksFinished = true;
		}
		/* get the state of user button */
		isButtonPressed = MyButton;

		/* detection for Squats */
		if (isButtonPressed 
			&& sitUpsFinished 
			&& pushUpsFinished 
			&& jumpJacksFinished 
			&& !squartsFinished) {
			/* LED4 turns on, reminding user to do Squarts. */
			MyLED4 = ON;
			/* different with free mode, no presampling process, initVal is 0 */
			countS(0);
			squartsFinished = true;
			/* when all finished, blink all leds and wait for user button interrupt to return */
			while (MyButton != ON) {
				MyLED3 = ON;
				MyLED5 = ON;
				MyLED6 = ON;
				MyLED4 = ON;
				thread_sleep_for(SHORT_TIME);
				MyLED3 = OFF;
				MyLED5 = OFF;
				MyLED6 = OFF;
				MyLED4 = OFF;
				thread_sleep_for(SHORT_TIME);
			}
			return;
		}

		
	}
}


int main() {
	/* check detection of the accelerometer */
	while(acc.Detect() != 1) {
        printf("Could not detect Accelerometer\n\r");
		MyLED4 = 1;
		wait_ms(200);
    }

	while(1) {
		/* Waiting for user button interrupt. */
		waitingLight();

		/* Wait for 3 seconds, to detach any vibrations. */
		wait_ms(LONG_TIME);

		/* Free to choose any exercise. */
		freeToExercise();
		isButtonPressed = false;

		wait_ms(LONG_TIME);

		/* Rountined exercise */
		routinedExercise();
		isButtonPressed = false;
	}
}
