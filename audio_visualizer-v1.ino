/*
 * ECE 341: Junior Design Accelerted Project 2
 * 
 * DFT Audio Visulizer
 * 
 * Author: Tristan Luther
 * Date: 10/29/2019
 * Purpose: Firmware for the DFT Audio
 * Visualizer
 * 
 */

//Macro Definitions for LED Matrix
#define ROW_1 2
#define ROW_2 3
#define ROW_3 4
#define ROW_4 5
#define ROW_5 6
#define ROW_6 7
#define ROW_7 8
#define ROW_8 9

#define COL_1 10
#define COL_2 11
#define COL_3 12
#define COL_4 13
#define COL_5 A1
#define COL_6 A2
#define COL_7 A3
#define COL_8 A4

//Array associations for matrix outputs
const byte rows[] = {
    ROW_1, ROW_2, ROW_3, ROW_4, ROW_5, ROW_6, ROW_7, ROW_8
};
const byte col[] = {
  COL_1,COL_2, COL_3, COL_4, COL_5, COL_6, COL_7, COL_8
};

//Fast Fourier Transform Library
#include <arduinoFFT.h>

//Declare Analog Signal Pin
uint8_t inSigPin = A0;

//Create FFT object and create variables for sampling
arduinoFFT FFT = arduinoFFT();
const uint16_t samples = 64;
const double samplingFrequency = 100; //Hz, must be less than 10000 due to ADC on ATMega168

unsigned int sampling_period_us; //Microsecond equivalent for the sampling period
unsigned long microseconds; 

double real[samples]; //Array for collecting real domain samples
double imag[samples]; //Array for collecting complex domain samples (Not used but required by FFT function)

uint8_t avgSig[8]; //Array that will contain the averaged FFT output, this will be sent to the display
long avg; //Intermediate value used for averaging

/*
 * The averageSignal function will take the inital real array after the FFT compute call and average
 * the FFT signal into an array that can be displayed on the 8x8 LED Matrix
 */
void averageSignal(){
  //Compress the data into an array of length 8 by averaging in groups of 16
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      //Sum for the average, taking only positive values
       avg += real[(8*i)+j];
    }
    //Map the average value simplified from 128 samples down to a scale of 0 - 7 for the output
    avgSig[i] = avg/8;
  }
  avg = 0;
}

/* 
 *  The matrixRaster function will raster though the LED matrix rows then columns and output to the 
 *  diplay based on the average FFT values found in the previous functions 
 */
void matrixRaster(){
   for(uint8_t i = 0; i < 8; i++){
    avgSig[i] = constrain(avgSig[i], 0, 80);
    avgSig[i] = map(avgSig[i], 0, 80, 0, 8);
    digitalWrite(col[i], LOW);
    if(avgSig[i] > 8){
      for(uint8_t j = 0; j < 8; j++){
        digitalWrite(rows[j], HIGH);
      }
    }
    else{
      for(uint8_t j = 0; j < avgSig[i]; j++){
        digitalWrite(rows[j], HIGH);
      }
    }
    delayMicroseconds(400);
    for(uint8_t k = 0; k < 8; k++){
      digitalWrite(rows[k], LOW);
    }
    digitalWrite(col[i], HIGH);
  }
}


void setup() {
  //Initalize the I/O & Begin Serial
  Serial.begin(9600);
  //Be sure the arduino is using the default(5V) analog reference 
  analogReference(DEFAULT);
  //Calculate the sampling period in microseconds from the above sampling frequency
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  //Initalize the LED Matrix outputs
  for(int i = 2; i < 14; i++){
    pinMode(i, OUTPUT);
  }
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
}

void loop() {
  //Sample the audio
  for(uint16_t i = 0; i < samples; i++){
    real[i] = ((analogRead(inSigPin)*1) - 512)/8;
    imag[i] = 0;
  }
  //Compute the Discrete Fouier Transform to get the frequency domain equivalent
  FFT.Compute(real, imag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, samples);
  //Average the signal for the 8x8 display
  averageSignal();
  //Raster over the positions on the martix and output where needed
  matrixRaster();
}


