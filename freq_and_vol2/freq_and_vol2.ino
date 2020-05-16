/****************************************
Example Sound Level Sketch for the 
Adafruit Microphone Amplifier
****************************************/
//#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ALLOW_INTERRUPTS 0
#include "FastLED.h" 
#include "arduinoFFT.h"

#define NUM_LEDS 6
#define DATA_PIN 5

//fft
//
#define SAMPLES 128           //Must be a power of 2
#define SAMPLING_FREQUENCY 5000 //Hz, must be less than 10000 due to ADC
//
//

//volume
//
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
//
//

//smoothing
//
const int numReadings = 10;
const int numReadingsPeak = 4;

unsigned long period_average_reset = 150000;
unsigned long time_now_average_reset = 0;

int peaks[numReadingsPeak];
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int peakTotal = 0;
int average = 0;                // the average
unsigned int peakToPeak;

int inputPin = A0;
//
//

int f = 0;
int val = 175;
CRGB leds[NUM_LEDS];

//fft
//
arduinoFFT FFT = arduinoFFT();
unsigned int sampling_period_us;
unsigned long microseconds;

int average_low = 0;
int average_high = 0;
bool  first = true;

struct HSV {
  int hue;
  int sat;
  int val;
};
 
double vReal[SAMPLES];
double vImag[SAMPLES];

HSV colorLED[150];
HSV black;

int redpin = 11; //select the pin for the red LED
int greenpin =10;// select the pin for the green LED
int bluepin =9; // select the pin for the  blue LED
//
//
void setup() 
{

  pinMode(redpin, OUTPUT); pinMode(bluepin, OUTPUT); pinMode(greenpin, OUTPUT);
   Serial.begin(9600);
   FastLED.addLeds<WS2811, DATA_PIN, GRB>(leds, NUM_LEDS);

   //fft
   //
   randomSeed(98155);
   //
   //
   
   //smoothing
   //
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
   //
   //

   //fft
   //
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
    delay(2000);
//    primary_loud = analogRead(0);

   black.hue = 0;
   black.sat = 0;
   black.val = 0;
   //
   //
}
 
 
void loop() 
{

  double peak = getPeak();
  
  peakToPeak = getPeakToPeak();
  unsigned int peakToPeakAverage = getAveragePeak(peakToPeak);
   
  getAverage(peak);

  getAverageRanges();

  
  delay(1); 

  int option = 3;

  int newAverage = map(average, average_low, average_high, 0, 255);
  int newPeak = map(peakToPeak, 1, 800, 128, 192);
  int newPeakAverage = map(peakToPeakAverage, 1, 800, 128, 192);
   for(int j = 0; j < NUM_LEDS; j++){
    switch(option){
      case 0:
        leds[j] = CHSV(newPeakAverage, 255, 255);
        break;
      case 1:
        leds[j] = CHSV(newPeak, 255, 255);
        break;
      case 2:
        leds[j] = CHSV(newAverage, 255, 255);
        break;
      case 3:
        newPeakAverage = map(peakToPeakAverage, 1, 800, 200, 255);
        leds[j] = CHSV(newAverage, 255, peakToPeakAverage);
        break;
      case 4:
        newPeakAverage = map(peakToPeakAverage, 1, 800, 0, 255);
        leds[j] = CHSV(newPeakAverage, 255, newPeakAverage);
        break;
           
    }
   }
   FastLED.show();

  analogWrite(11, leds[0].red); analogWrite(10, leds[0].green); analogWrite(9, leds[0].blue);
  Serial.print(newPeakAverage);
  Serial.print(",");
  Serial.print(newPeak);
  Serial.print(",");
  Serial.println(newAverage);
//  Serial.print(",");
//  Serial.print(average_low);
//  Serial.print(",");
//  Serial.println(average_high);
//  Serial.print(",");
//  Serial.println(peakToPeak);
}


//gets the peakToPeak
unsigned int getPeakToPeak(){
  if(readIndex%2 == 1){
    return peakToPeak;
  }
  
  unsigned long startMillis= millis();  // Start of sample window
  peakToPeak = 0;   // peak-to-peak level
  
  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;
  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow){
      sample = analogRead(A0);
      if (sample < 1024){  // toss out spurious readings
         if (sample > signalMax){
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin){
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
   return peakToPeak;
}

int getAveragePeak(double peakToPeak){
  peakTotal = peakTotal - peaks[readIndex%numReadingsPeak];
  // read from the sensor:
  peaks[readIndex%numReadingsPeak] = peakToPeak;
  // add the reading to the total:
  peakTotal = peakTotal + peaks[readIndex%numReadingsPeak];
  // calculate the average:
  average = peakTotal / numReadingsPeak;
  
  return average;
}

//gets the average value
void getAverage(double peak){
    total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = peak;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;
}

void getAverageRanges(){
  //sets the range for the average values so that the range of colors is greater
  if(first){
    average_low = average;
    average_high = average;
    first = false;
  }else{
    if(average_low > average){
      average_low = average;
    }
    if(average_high < average){
      average_high = average;
    }
  }
//  if(millis() - time_now_average_reset >  period_average_reset){
//       time_now_average_reset = millis();
//       first = true;
//  }  
}

double getPeak(){
  for(int i=0; i<SAMPLES; i++){
      microseconds = micros();    //Overflows after around 70 minutes!
      
      vReal[i] = analogRead(0);
      vImag[i] = 0;
      
      while(micros() < microseconds +  sampling_period_us){
      }
    }
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    return peak;
}
