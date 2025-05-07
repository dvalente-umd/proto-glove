/*
  Example using a potentiometer to control the amplitude of a sinewave
  with Mozzi sonification library.

  Demonstrates the use of Oscil to play a wavetable, and analog input for control.

  This example goes with a tutorial on the Mozzi site:
  http://sensorium.github.io/Mozzi/learn/introductory-tutorial/

  The circuit:
    Audio output on digital pin 9 on a Uno or similar, or
    DAC/A14 on Teensy 3.1, or
    check the README or http://sensorium.github.io/Mozzi/

  Potentiometer connected to analog pin 0:
     Center pin of the potentiometer goes to the analog pin.
     Side pins of the potentiometer go to +5V and ground

 +5V ---|
              /
  A0 ----\  potentiometer
              /
 GND ---|

   Mozzi documentation/API
   https://sensorium.github.io/Mozzi/doc/html/index.html

   Mozzi help/discussion/announcements:
   https://groups.google.com/forum/#!forum/mozzi-users

   Copyright 2013-2024 Tim Barrass and the Mozzi Team

   Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later.
*/

#include <Mozzi.h>
#include <Oscil.h> // oscillator template
#include <tables/sin2048_int8.h> // sine table for oscillator
#include <tables/saw2048_int8.h> // saw wave table for oscillator
#include <tables/triangle2048_int8.h> // triangle wave table
#include <Wire.h>
#include "Adafruit_Trellis.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <AutoMap.h>
#include <utility/imumaths.h>
#include <math.h>
#include <Smooth.h>
#include <ResonantFilter.h>

#define SERIAL_DEBUG true
#define BNO_ENABLED true
// set to however many you're working with here, up to 8
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
// Connect Trellis Vin to 5V and Ground to ground.
// Connect the INT wire to pin #A2 (can change later!)
#define INTPIN A5
#define PI 3.1415
// Connect I2C SDA pin to your Arduino SDA line
// Connect I2C SCL pin to your Arduino SCL line
// All Trellises share the SDA, SCL and INT pin!
// Even 8 tiles use only 3 wires max

const char THUMB = 4; // set the input for the knob to analog pin 0
const char INDEX = 3;
const char MIDDLE = 2;
const char RING = 1;

// desired intensity max and min, for AutoMap, note they're inverted for reverse dynamics
const float MAX_INTENSITY = 700;
const float MIN_INTENSITY = 10;

// desired mod speed max and min, for AutoMap, note they're inverted for reverse dynamics
const float MAX_MOD_SPEED = 10000;
const float MIN_MOD_SPEED = 1;


//carrier1 values for fm_synth: 22-440
float MIN_CARRIER1_FREQ = 22;
float MAX_CARRIER1_FREQ = 440;
float MIN_CARRIER2_FREQ = 22;
float MAX_CARRIER2_FREQ = 440;
float MIN_CARRIER3_FREQ = 22;
float MAX_CARRIER3_FREQ = 440;

int mod_ratio = 5;
long fm_intensity;

float smoothness = 0.95f;

uint8_t resonance = 0; // range 0-255, 255 is most resonant

// to convey the volume level from updateControl() to updateAudio()
int volume;
int index_read;
int middle_read;
int ring_read;

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier1(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier2(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier3(SIN2048_DATA);


// Effect Oscillators
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> LFO1(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> LFO2(SIN2048_DATA);

// Filters for Phaser Effect
ResonantFilter<NOTCH> rf; // Notch filter
ResonantFilter<NOTCH> rf2;

// smoothing for intensity to remove clicks on transitions
Smooth <long> aSmoothIntensity(smoothness);

/*Finger Ranges*/
int index_range[2] = {1180, 1860};
int middle_range[2] = {745, 1780};
int ring_range[2] = {940, 1880};

//Maps
AutoMap kMapCarrier1Freq(index_range[0],index_range[1],MIN_CARRIER1_FREQ,MAX_CARRIER1_FREQ); //Index
AutoMap kMapCarrier2Freq(middle_range[0],middle_range[1],MIN_CARRIER2_FREQ,MAX_CARRIER2_FREQ); //Middle
AutoMap kMapCarrier3Freq(ring_range[0],ring_range[1],MIN_CARRIER3_FREQ,MAX_CARRIER3_FREQ); //Ring

AutoMap kMapIntensity(-90,30,MIN_INTENSITY,MAX_INTENSITY); //Maps to y rotation
AutoMap kMapModSpeed(-110,20,MIN_MOD_SPEED,MAX_MOD_SPEED); //Maps to z rotation

AutoMap FilterMapY(-90, 30, 0, 255);
AutoMap FilterMapZ(-110, 20, 0, 255);

/*  Keypad Globals  */
int toggleArray[numKeys] = { 0 };

Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis = Adafruit_TrellisSet(&matrix0);

uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
double freq = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;

// How long it should take for the buttons to update
long delay_time_buttons = 30;
long delay_time_imu = 150;

float x;
float y;
float z;
sensors_event_t orientationData;

unsigned long currentMillis;

float freq_1_hold;

float chord_map_index[6][2] = {
  {261.63f, 277.18f}, {293.67f, 311.13f}, {329.63f, 349.23f},
  {369.99f, 392.0f}, {415.3f, 440.0f}, {466.16f, 493.88f}
};

float chord_map_middle[6][2] = {
  {329.63f, 349.23f}, {369.99f, 392.0f}, {415.3f, 440.0f},
  {466.16f, 493.88f}, {523.25f, 554.37f}, {587.33f, 622.25f}
};

float chord_map_ring[6][2] = {
  {392.0f, 415.3f}, {440.0f, 466.16f}, {493.88f, 523.25f},
  {554.37f, 587.33f}, {622.25f, 659.26f}, {698.46f, 739.99f}
};

uint8_t key_select;
uint8_t effect_select;
uint8_t toggle_freeRange;

void setup(){
  Serial.begin(9600); // for Teensy 3.1, beware printout can cause glitches
  //Serial.begin(115200); // set up the Serial output so we can look at the piezo values // set up the Serial output so we can look at the input values

  carrier1.setFreq(440);
  carrier2.setFreq(554.37f);
  carrier3.setFreq(659.26f);
  startMozzi(); // :))

  toggleArray[0] = 1;
  key_select = 0;
  effect_select = 0;

  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);
  trellis.begin(0x70);
  if(SERIAL_DEBUG) {
    while (!Serial) delay(10);
  }
  
  if (BNO_ENABLED && !bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
  bno.setExtCrystalUse(true);

}

// all control logic goes in here
void updateControl(){
  // read the variable resistor for volume. We specifically request only 8 bits of resolution, here, which
  // is less than the default on most platforms, but a convenient range to work with, where accuracy is not too important.
  volume = mozziAnalogRead<12>(THUMB);

  index_read = mozziAnalogRead<12>(INDEX);
  middle_read = mozziAnalogRead<12>(MIDDLE);
  ring_read = mozziAnalogRead<12>(RING);

  //Serial.print("Index: "); Serial.print(index_read); Serial.print("\tMiddle: "); Serial.print(middle_read); Serial.print("\tRing: "); Serial.println(ring_read);


  currentMillis = millis();
  if(currentMillis - previousMillis > delay_time_buttons){
    updateButtons();
    previousMillis = currentMillis;
  }
  if(BNO_ENABLED){
    if(currentMillis - previousMillis2 > delay_time_imu){
      updateIMU();
      previousMillis2 = currentMillis;
    }
  }

  if(toggleArray[0] == 1){
    carrier1.setTable(SIN2048_DATA);
    carrier2.setTable(SIN2048_DATA);
    carrier3.setTable(SIN2048_DATA);
  }
  if(toggleArray[1] == 1){
    carrier1.setTable(SAW2048_DATA);
    carrier2.setTable(SAW2048_DATA);
    carrier3.setTable(SAW2048_DATA);
  } 
  if(toggleArray[2] == 1){
    carrier1.setTable(TRIANGLE2048_DATA);
    carrier2.setTable(TRIANGLE2048_DATA);
    carrier3.setTable(TRIANGLE2048_DATA);
  }

  volume = map((int)volume, 2900, 3100, 0, 255);  

  chord_select();

  switch (effect_select){
      case 1:
        FM_synthesis();
        break;
      case 2:
        phaser_effect();
        break;
  }

  //phaser_effect();

  //FM_synthesis();

  

}


AudioOutput updateAudio(){

  //Serial.print("\n"); Serial.println(effect_select);
  uint8_t signal;

  /*switch (effect_select){
    case 0:
      signal = carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1;
      break;
      //return MonoOutput::from16Bit((carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1)*volume);
    case 1:
      long modulation = aSmoothIntensity.next(fm_intensity) * LFO1.next();
      signal = carrier1.phMod(modulation)>>1 + carrier2.phMod(modulation)>>1 + carrier3.phMod(modulation)>>1;
      break;
      //return MonoOutput::fromNBit(17, (carrier1.phMod(modulation)>>1 + carrier2.phMod(modulation)>>1 + carrier3.phMod(modulation)>>1)*volume);
    case 2:
      Serial.println("here");
      signal = rf2.next((char)rf.next(carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1));
      break;
      //char filtsig = rf2.next((char)rf.next(carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1));
      //return MonoOutput::fromNBit(16, (volume*(filtsig)));
  }*/

  if(effect_select == 0){
    signal = carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1;
  }
  else if(effect_select == 1){
    long modulation = aSmoothIntensity.next(fm_intensity) * LFO1.next();
    signal = carrier1.phMod(modulation)>>1 + carrier2.phMod(modulation)>>1 + carrier3.phMod(modulation)>>1;
  }
  else{
    signal = rf2.next((char)rf.next(carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1));
  }

  //char filtsig = rf2.next((char)rf.next(carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1));
  return MonoOutput::fromNBit(16, (volume*(signal)));

  //char filtsig = rf2.next((char)rf.next(carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1));
  //return MonoOutput::fromNBit(16, (volume*(filtsig)));

  //return MonoOutput::fromAlmostNBit(8, carrier1.next()>>1 + carrier2.next()>>1 + carrier3.next()>>1);

  //long modulation = aSmoothIntensity.next(fm_intensity) * LFO1.next();
  //return MonoOutput::from16Bit((carrier1.phMod(modulation)>>1 + carrier2.phMod(modulation)>>1 + carrier3.phMod(modulation)>>1)*volume); // 8 bit * 8 bit gives 16 bits value*/
  
}

void phaser_effect(){
  float lfo_speed_1 = (float)FilterMapY(y)/32;
  float lfo_speed_2 = (float)FilterMapZ(z)/32;

  Serial.print("LFO1 Speed: "); Serial.print(lfo_speed_1); Serial.print("LFO2 Speed: "); Serial.println(lfo_speed_2);


  LFO1.setFreq(lfo_speed_1);
  LFO2.setFreq(lfo_speed_2);

  // map the modulation into the filter range (0-255), corresponds with 0-MOZZI_AUDIO_RATE/(sqrt(2)*pi) Hz
  byte cutoff_freq = 100 + LFO1.next()/2;
  byte cutoff_freq2 = 100 + LFO2.next()/2;
  rf.setCutoffFreqAndResonance(cutoff_freq, resonance);
  rf2.setCutoffFreqAndResonance(cutoff_freq2, resonance);
}

void chord_select(){
  if(index_read <= (index_range[0]+index_range[1])>>1){
    freq_1_hold = chord_map_index[key_select][0];
    carrier1.setFreq(chord_map_index[key_select][0]);
  }
  else{
    freq_1_hold = chord_map_index[key_select][1];
    carrier1.setFreq(chord_map_index[key_select][1]);
  }

  if(middle_read <= (middle_range[0]+middle_range[1])>>1){
    carrier2.setFreq(chord_map_middle[key_select][0]);
  }
  else{
    carrier2.setFreq(chord_map_middle[key_select][1]);
  }

  if(ring_read <= (ring_range[0]+ring_range[1])>>1){
    carrier3.setFreq(chord_map_ring[key_select][0]);
  }
  else{
    carrier3.setFreq(chord_map_ring[key_select][1]);
  }

}

void FM_synthesis(){
  //int pitch_set = kMapCarrier1Freq(index_read); //for freerange frequencies
  // print the value to the Serial monitor for debugging

  int mod_freq = freq_1_hold * mod_ratio;

  //carrier1.setFreq(pitch_set); //for freerange frequencies
  LFO1.setFreq(mod_freq);

  //int fm_read = mozziAnalogRead(MIDDLE);
  //int speed_read = mozziAnalogRead(RING);

  //Serial.print("Thumb read: "); Serial.print(volume); Serial.print("\tIndex read: "); Serial.print(pitch);
  //Serial.print("\tMiddle read: " ); Serial.print(fm_read); Serial.print("\tRing read: "); Serial.println(speed_read);


  int fm_calibrated = kMapIntensity(y);
  float mod_speed = (float)kMapModSpeed(z)/1000;

  fm_intensity = ((long)fm_calibrated * (LFO2.next()+128))>>8; // shift back to range after 8 bit multiply

  //Serial.print("Y: "); Serial.print((int)y); Serial.print("\tZ: "); Serial.println((int)z);
  /*Serial.print("FM calibrated: "); Serial.print(fm_calibrated); Serial.print("\tFM intensity: "); Serial.print(fm_intensity);
  Serial.print("\tMod Speed: "); Serial.println(mod_speed);*/

  LFO2.setFreq(mod_speed);

}

void updateIMU() {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.type == SENSOR_TYPE_ORIENTATION) {
    x = orientationData.orientation.x;
    y = orientationData.orientation.y;
    z = orientationData.orientation.z;
    /*if(SERIAL_DEBUG) {
      Serial.print("Orient:");
      Serial.print("\tx= ");
      Serial.print(x);
      Serial.print(" |\ty= ");
      Serial.print(y);
      Serial.print(" |\tz= ");
      Serial.println(z);
    }*/
  }
}


void updateButtons() {
    // If a button was just pressed or released...
  if (trellis.readSwitches()) {
    // go through every button
    for (uint8_t i = 0; i < numKeys; i++) {
      // if it was pressed...
      if (trellis.justPressed(i)) {
        // Alternate the LED
        toggleArray[i] = !toggleArray[i];
        // mutex buttons, diffirent waveforms
        if(SERIAL_DEBUG) {
          Serial.print(i);
          if(toggleArray[i] == 1) {
            Serial.println(" turned on");
          } else {
            Serial.println(" turned off");
          }
        }
        if(i == 0) {
          toggleArray[1] = 0;
          toggleArray[2] = 0;
          if(SERIAL_DEBUG) {
            Serial.println("0 on, 1 and 2 off");
          }
        }
        else if(i == 1) {
          toggleArray[0] = 0;
          toggleArray[2] = 0;
          if(SERIAL_DEBUG) {
            Serial.println("1 on, 0 and 2 off");
          }
        }
        else if(i == 2) {
          toggleArray[0] = 0;
          toggleArray[1] = 0;
          if(SERIAL_DEBUG) {
            Serial.println("2 on, 0 and 1 off");
          }
        }
        else if(i == 4){
          key_select = 0;
        }
        else if(i == 5){
          key_select = 1;
        }
        else if(i == 6){
          key_select = 2;
        }
        else if(i == 7){
          key_select = 3;
        }
        else if(i == 8){
          key_select = 4;
        }
        else if(i == 9){
          key_select = 5;
        }
        else if(i == 12){
          effect_select = 0;
        }
        else if(i == 13){
          effect_select = 1;
        }
        else if(i == 14){
          effect_select = 2;
        }

      }
    }
  }
}

/*float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}*/

void loop(){
  audioHook(); // required here

}