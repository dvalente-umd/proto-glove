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

#define SERIAL_DEBUG true
#define BNO_ENABLED true
// set to however many you're working with here, up to 8
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
int toggleArray[numKeys] = { 0 };
// Connect Trellis Vin to 5V and Ground to ground.
// Connect the INT wire to pin #A2 (can change later!)
#define INTPIN A5

#define PI 3.1415
// Connect I2C SDA pin to your Arduino SDA line
// Connect I2C SCL pin to your Arduino SCL line
// All Trellises share the SDA, SCL and INT pin!
// Even 8 tiles use only 3 wires max

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier(SIN2048_DATA);

// Effect Oscillators
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> modulator(SIN2048_DATA);
Oscil <SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> intensityMod(SIN2048_DATA);


int mod_ratio = 5;
long fm_intensity;
// smoothing for intensity to remove clicks on transitions
float smoothness = 0.95f;
Smooth <long> aSmoothIntensity(smoothness);


Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis = Adafruit_TrellisSet(&matrix0);

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

const float MIN_CARRIER_FREQ = 22;
const float MAX_CARRIER_FREQ = 440;

//Maps
AutoMap kMapCarrierFreq(1180,1860,MIN_CARRIER_FREQ,MAX_CARRIER_FREQ);
AutoMap kMapIntensity(-90,30,MIN_INTENSITY,MAX_INTENSITY);
AutoMap kMapModSpeed(-110,20,MIN_MOD_SPEED,MAX_MOD_SPEED);

// to convey the volume level from updateControl() to updateAudio()
int volume;
int pitch;


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


void setup(){
  Serial.begin(9600); // for Teensy 3.1, beware printout can cause glitches
  //Serial.begin(115200); // set up the Serial output so we can look at the piezo values // set up the Serial output so we can look at the input values

  carrier.setFreq(440);
  startMozzi(); // :))

  toggleArray[0] = 1;

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
  pitch = mozziAnalogRead<12>(INDEX);




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

  if(toggleArray[0] == 1) carrier.setTable(SIN2048_DATA);
  if(toggleArray[1] == 1) carrier.setTable(SAW2048_DATA);
  if(toggleArray[2] == 1) carrier.setTable(TRIANGLE2048_DATA);

  volume = map((int)volume, 2900, 3100, 0, 255);

  FM_synthesis();

  /*if(SERIAL_DEBUG){
    Serial.print("volume = ");
    Serial.println((int)volume);
    Serial.print("Pitch =  "); Serial.println((int)pitch);
  }*/

  //int freq_set = (int)pitch*3-62.5;

}


AudioOutput updateAudio(){
  long modulation = aSmoothIntensity.next(fm_intensity) * modulator.next();
  return MonoOutput::from16Bit(carrier.phMod(modulation)*volume); // 8 bit * 8 bit gives 16 bits value
}

void FM_synthesis(){
  int pitch_set = kMapCarrierFreq(pitch);
  // print the value to the Serial monitor for debugging

  int mod_freq = pitch_set * mod_ratio;

  carrier.setFreq(pitch_set);
  modulator.setFreq(mod_freq);

  //int fm_read = mozziAnalogRead(MIDDLE);
  //int speed_read = mozziAnalogRead(RING);

  //Serial.print("Thumb read: "); Serial.print(volume); Serial.print("\tIndex read: "); Serial.print(pitch);
  //Serial.print("\tMiddle read: " ); Serial.print(fm_read); Serial.print("\tRing read: "); Serial.println(speed_read);


  int fm_calibrated = kMapIntensity(y);
  float mod_speed = (float)kMapModSpeed(z)/1000;

  fm_intensity = ((long)fm_calibrated * (intensityMod.next()+128))>>8; // shift back to range after 8 bit multiply

  //Serial.print("Y: "); Serial.print((int)y); Serial.print("\tZ: "); Serial.println((int)z);
  /*Serial.print("FM calibrated: "); Serial.print(fm_calibrated); Serial.print("\tFM intensity: "); Serial.print(fm_intensity);
  Serial.print("\tMod Speed: "); Serial.println(mod_speed);*/

  intensityMod.setFreq(mod_speed);

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
        if(i == 1) {
          toggleArray[0] = 0;
          toggleArray[2] = 0;
          if(SERIAL_DEBUG) {
            Serial.println("1 on, 0 and 2 off");
          }
        }
        if(i == 2) {
          toggleArray[0] = 0;
          toggleArray[1] = 0;
          if(SERIAL_DEBUG) {
            Serial.println("2 on, 0 and 1 off");
          }
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