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

#include <arduinoFFT.h>

//Specifications & I/O for Hardware
uint8_t inSigPin = A0;
uint8_t clkPin = 10;
uint8_t resetPin = 11;

//Sampling Audio
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 128;
const double samplingFrequency = 100; //Hz, must be less than 10000 due to ADC

unsigned int sampling_period_us;
unsigned long microseconds;

double real[samples];
double imag[samples];

uint8_t avgSig[8];
long avg;
int maxValue = 1000;

void setup() {
  //Initalize the I/O & Begin Serial
  Serial.begin(9600);
  sampling_period_us = round(1000000*(1.0/samplingFrequency));

  for(int i = 2; i < 10; i++){
    pinMode(i, OUTPUT);
  }
  pinMode(clkPin, OUTPUT);
  pinMode(resetPin, OUTPUT);
}

void loop() {
  //Sample the audio
  for(uint16_t i = 0; i < samples; i++){
    real[i] = analogRead(inSigPin);
    imag[i] = 0;
  }
  //Compute the Discrete Fouier Transform to get the frequency domain equivalent
  FFT.Compute(real, imag, samples, FFT_FORWARD);
  FFT.ComplexToMagnitude(real, imag, samples); /* Compute magnitudes */

  //Compress the data into an array of length 8 by averaging in groups of 16
  for(int i = 0; i < 8; i++){
    for(int j = 0; j < 8; j++){
      //Sum for the average, taking only positive values
       avg += real[(8*i)+j];
    }
    //Map the average value simplified from 128 samples down to a scale of 0 - 7 for the output
    avgSig[i] = map((avg/16), 0, maxValue, 0, 7);  
  }
  avg = 0;

  for(uint8_t i = 0; i < 8; i++){
    digitalWrite(clkPin, HIGH);
    digitalWrite(clkPin, LOW);
    
    for(uint8_t j = 0; j < avgSig[i]; j++){
      digitalWrite(j, HIGH);
    }
  }
  digitalWrite(resetPin, HIGH);
  digitalWrite(resetPin, LOW);
  
}

