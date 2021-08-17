#include <FastLED.h>
#include <arduinoFFT.h>

// ESP32 FastLED FFT Dual Core Audio sampling
// This code shows a method for using both cores of the esp32 for audio sampling and updating an LED strip
// Core 0 of the esp32 Samples Audio Data, while core 1 updates the strip values and sends out the LED data to the strip
// Band Values are shared between the two tasks and managed using a mutually exclusive semaphore
// The LED effect is a very basic implementation of a bar which will expand and contract depending on how present the bass notes are
// This code adapts the Audio Sampling code from Scott Marley in his VU Meter Spectrum Analyzer Tutorial, If you are unfamilier with audio sampling and FFT for use with FastLED, this is a good place to start.
// Scott Marley Video -  https://www.youtube.com/watch?v=Mgh2WblO5_c&ab_channel=ScottMarley
// Scott Marley github spectrum analyzer link - github - https://github.com/s-marley/ESP32_FFT_VU
//
// Feel free to use this code for your projects or play around with it to see what you can create!
// Garrick Hogrebe August 2021


#define AUDIO_IN_PIN    14             // Audio signal pin
#define LED_PIN         15             // LED data strip pin

#define SAMPLES         1024          // Must be a power of 2
#define SAMPLING_FREQ   40000         // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define COLOR_ORDER     GRB           // If colours look wrong, play with this
#define CHIPSET         WS2812B       // LED strip type
#define NUM_BANDS       16            // To change this, you will need to change the bunch of if statements describing the mapping from bins to bands

#define NOISE           4000           // Used as a crude noise filter, values below this are ignored. Try lowering this if you dont pick up audio data.

#define NUM_LEDS       167           // Total number of LEDs
#define BRIGHTNESS      100           //Brightness
#define FRAMES_PER_SECOND 120         //Used to insert delay

TaskHandle_t samplingTask;
SemaphoreHandle_t bandLock;

// Sampling and FFT stuff
unsigned int sampling_period_us;             
int bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // The length of these arrays must be >= NUM_BANDS
int gbandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double vReal[SAMPLES];
double vImag[SAMPLES];
unsigned long newTime;
arduinoFFT FFT = arduinoFFT(vReal, vImag, SAMPLES, SAMPLING_FREQ);

//Create our LED Array
CRGB leds[NUM_LEDS];

//Used to adjust hue of our beat bar
uint8_t colorTimer = 0;

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();

  //Calculate Sampling Period
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQ));
  bandLock = xSemaphoreCreateMutex();

  //Begin our audio sampling task on core 0
  xTaskCreatePinnedToCore(samplingLoop, "Sampling Task", 10000, NULL, 0, &samplingTask, 0);
}

int height = 0;

void samplingLoop(void * parameter){//Our function to sample audio data and run repeatedly on core 0
 for(;;){
  // Reset bandValues[]
  for (int i = 0; i<NUM_BANDS; i++){
    bandValues[i] = 0;
  }

  // Sample the audio pin
  for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    vReal[i] = analogRead(AUDIO_IN_PIN); // A conversion takes about 9.7uS on an ESP32
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* chill */ }
  }

  // Compute FFT
  FFT.DCRemoval();
  FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(FFT_FORWARD);
  FFT.ComplexToMagnitude();

  // Analyse FFT results
  for (int i = 2; i < (SAMPLES/2); i++){       // Don't use sample 0 and only first SAMPLES/2 are usable. Each array element represents a frequency bin and its value the amplitude.
    if (vReal[i] > NOISE) {                    // Add a crude noise filter

    //16 bands, 12kHz top band
      if (i<=2 )           bandValues[0]  += (int)vReal[i];
      if (i>2   && i<=3  ) bandValues[1]  += (int)vReal[i];
      if (i>3   && i<=5  ) bandValues[2]  += (int)vReal[i];
      if (i>5   && i<=7  ) bandValues[3]  += (int)vReal[i];
      if (i>7   && i<=9  ) bandValues[4]  += (int)vReal[i];
      if (i>9   && i<=13 ) bandValues[5]  += (int)vReal[i];
      if (i>13  && i<=18 ) bandValues[6]  += (int)vReal[i];
      if (i>18  && i<=25 ) bandValues[7]  += (int)vReal[i];
      if (i>25  && i<=36 ) bandValues[8]  += (int)vReal[i];
      if (i>36  && i<=50 ) bandValues[9]  += (int)vReal[i];
      if (i>50  && i<=69 ) bandValues[10] += (int)vReal[i];
      if (i>69  && i<=97 ) bandValues[11] += (int)vReal[i];
      if (i>97  && i<=135) bandValues[12] += (int)vReal[i];
      if (i>135 && i<=189) bandValues[13] += (int)vReal[i];
      if (i>189 && i<=264) bandValues[14] += (int)vReal[i];
      if (i>264          ) bandValues[15] += (int)vReal[i];
    }
  }

  //Copy values to global - By using another array to hold these values our LED writing task on core 1 doesn't have to wait for this task to finish sampling. It will only have to wait for these values to be copied which is much faster.
  xSemaphoreTake(bandLock, portMAX_DELAY);
  for (int x = 0; x < NUM_BANDS; x++){
    gbandValues[x] = bandValues[x];
  }
  xSemaphoreGive(bandLock);
  }
}


int prev = 0; //used for smoothing out the bar with weighted average
int total = 0;

void loop() {//Loop is defualt to core 1 on esp32

//Fade the leds - Leds farther from the center are faded more
  for(int i = 0; i < NUM_LEDS; i++){
    fadeToBlackBy(leds, NUM_LEDS, 50);
    //fadeToBlackBy(&leds[i] , 1, abs(i - (NUM_LEDS/2))/5 + 8);
  }    

  //Take the lock
  xSemaphoreTake(bandLock, portMAX_DELAY);

  //Figure out how much total "Volume" is present by adding all band values together
  total = 0;
  for (int x = 0; x < NUM_BANDS; x++){
    total += gbandValues[x];
  }
  
  //We are done with bandValues - Free the lock
  xSemaphoreGive(bandLock);

  //Adjust this constant to change how far out the bar goes
  height = total/40000;

  //Make sure the bar does not go outside our LED array
  if (height > (NUM_LEDS - 1)/2){
    height = (NUM_LEDS - 1)/2;
  }

  //Weighted Average to smooth out how the bar grows and shrinks
  height = (prev*2 + 3*height)/5;
  prev = height;

  //Write the leds
  for (int x = 0; x < height; x++){
    leds[(NUM_LEDS)/2 + x] = CHSV(colorTimer,255,255);
    leds[(NUM_LEDS)/2 - x] = CHSV(colorTimer,255,255);    
  }
      
  // Cycle the color slowly
  EVERY_N_MILLISECONDS(800) {
    colorTimer++;
  }


  FastLED.show();
  FastLED.delay(1000/FRAMES_PER_SECOND);
}
