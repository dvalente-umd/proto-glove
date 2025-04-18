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
#include <utility/imumaths.h>
#include <math.h>

// use: Oscil <table_size, update_rate> oscilName (wavetable), look in .h file of table #included above
Oscil <SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier(SIN2048_DATA);

//Oscil <SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw(SAW2048_DATA);
//Oscil <TRIANGLE2048_NUM_CELLS, MOZZI_AUDIO_RATE> aTri(TRIANGLE2048_DATA);

Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis = Adafruit_TrellisSet(&matrix0);

const char INPUT_PIN = 0; // set the input for the knob to analog pin 0
const char FREQ_IN = 1;

// set to however many you're working with here, up to 8
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
int toggleArray[numKeys] = { 0 };
// Connect Trellis Vin to 5V and Ground to ground.
// Connect the INT wire to pin #A2 (can change later!)
#define INTPIN A2
// Connect I2C SDA pin to your Arduino SDA line
// Connect I2C SCL pin to your Arduino SCL line
// All Trellises share the SDA, SCL and INT pin!
// Even 8 tiles use only 3 wires max

// to convey the volume level from updateControl() to updateAudio()
byte volume;
byte pitch;

//byte pressed;



uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
double freq = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
unsigned long previousMillis = 0;

// How long it should take for the buttons to update
long delay = 30;

unsigned long currentMillis;


void setup(){
  //Serial.begin(9600); // for Teensy 3.1, beware printout can cause glitches
  Serial.begin(115200); // set up the Serial output so we can look at the piezo values // set up the Serial output so we can look at the input values

  carrier.setFreq(440);
  startMozzi(); // :))

  toggleArray[0] = 1;

  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);
  trellis.begin(0x70);
  while (!Serial) delay(10);
  /*
  if (!bno.begin())
  {
    //There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  */
  bno.setExtCrystalUse(true);

}

// all control logic goes in here
void updateControl(){
  // read the variable resistor for volume. We specifically request only 8 bits of resolution, here, which
  // is less than the default on most platforms, but a convenient range to work with, where accuracy is not too important.
  volume = mozziAnalogRead<8>(INPUT_PIN);
  pitch = mozziAnalogRead<8>(FREQ_IN);
  currentMillis = millis();
  if(currentMillis - previousMillis > interval){
    updateButtons();
  }


  if(toggleArray[0] == 1) carrier.setTable(SIN2048_DATA);
  if(toggleArray[1] == 1) carrier.setTable(SAW2048_DATA);
  if(toggleArray[2] == 1) carrier.setTable(TRIANGLE2048_DATA);

  //volume = map((int)volume, 135, 255, 0, 255);
  int pitch_set = map((int)pitch, 0, 255, 220, 2000);
  // print the value to the Serial monitor for debugging
  Serial.print("volume = ");
  Serial.println((int)volume);
  Serial.print("Pitch =  "); Serial.println((int)pitch_set);

  //int freq_set = (int)pitch*3-62.5;
  carrier.setFreq(pitch_set);



}


AudioOutput updateAudio(){

  return MonoOutput::from16Bit((int)carrier.next() * volume); // 8 bit * 8 bit gives 16 bits value
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
        Serial.print(i);
        if(toggleArray[i] == 1) {
          Serial.println(" turned on");
        } else {
          Serial.println(" turned off");
        }
        if(i == 0) {
          toggleArray[1] = 0;
          toggleArray[2] = 0;
          Serial.println("0 on, 1 and 2 off");
        }
        if(i == 1) {
          toggleArray[0] = 0;
          toggleArray[2] = 0;
          Serial.println("1 on, 0 and 2 off");
        }
        if(i == 2) {
          toggleArray[0] = 0;
          toggleArray[1] = 0;
          Serial.println("2 on, 0 and 1 off");
        }
      }
    }
  }
}

void loop(){
  audioHook(); // required here

}