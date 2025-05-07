/*  Example of flanging,
    using Mozzi sonification library.

    Demonstrates AudioDelayFeedback.

    Circuit: Audio output on digital pin 9 on a Uno or similar, or
    DAC/A14 on Teensy 3.1, or
    check the README or http://sensorium.github.io/Mozzi/

   Mozzi documentation/API
   https://sensorium.github.io/Mozzi/doc/html/index.html

   Mozzi help/discussion/announcements:
   https://groups.google.com/forum/#!forum/mozzi-users

   Copyright 2013-2024 Tim Barrass and the Mozzi Team

   Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later.
*/

#define MOZZI_CONTROL_RATE 128 // Hz, powers of 2 are most reliable
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/triangle_analogue512_int8.h> // wavetable
#include <tables/saw_analogue512_int8.h>
#include <tables/triangle2048_int8.h> // wavetable
#include <tables/sin512_int8.h> // sine table for oscillator
#include <AudioDelayFeedback.h>
#include <mozzi_midi.h> // for mtof
#include <ResonantFilter.h>
#include <AutoMap.h>

Oscil<SAW_ANALOGUE512_NUM_CELLS, MOZZI_AUDIO_RATE> carrier(SIN512_DATA); // audio oscillator
Oscil<TRIANGLE2048_NUM_CELLS, MOZZI_CONTROL_RATE> kDelSamps(TRIANGLE2048_DATA); // for modulating delay time, measured in audio samples

AudioDelayFeedback <128> aDel;
ResonantFilter<LOWPASS> cf;

AutoMap CarrierMap(0, 255, 50, 200);
AutoMap DelayMap(0, 255, 0, 1000);

// the delay time, measured in samples, updated in updateControl, and used in updateAudio
byte del_samps;

byte volume; 

int8_t del_samp;

const int FREQ_IN = 0;
const int DELAY_IN = 1;
const int VOL_IN = 2;


void setup(){

  Serial.begin(115200);

  startMozzi();
  carrier.setFreq(1000);
  kDelSamps.setFreq(.163f); // set the delay time modulation frequency (ie. the sweep frequency)
  aDel.setFeedbackLevel(-110); // can be -128 to 127 original value -111

  cf.setCutoffFreqAndResonance(200, 0);
}


Q16n16 deltime;


void updateControl(){
  // delay time range from 0 to 127 samples, @ 16384 samps per sec = 0 to 7 milliseconds
  //del_samps = 64+kDelSamps.next();

  // delay time range from 1 to 33 samples, @ 16384 samps per sec = 0 to 2 milliseconds
  //del_samps = 17+kDelSamps.next()/8;

  //byte f1 = mozziAnalogRead<8>(freq_in);
  //int freq_map = map((int)f1, 0, 255, 220, 2000);

  //volume = mozziAnalogRead<8>(2);
  //byte f_read = mozziAnalogRead<8>(0);
  byte d_read = mozziAnalogRead<8>(1);

  //int freq_set = CarrierMap(f_read);

  float del_set = (float)DelayMap(d_read)/1000;

  //Serial.print("Carrier freq: "); Serial.print(freq_set); //Serial.print(" Delay freq: "); Serial.println(del_set);

  //carrier.setFreq(freq_set);
  kDelSamps.setFreq(del_set);

  //cf.setCutoffFreq(freq_set);

  deltime = Q8n0_to_Q16n16(17) + ((long)kDelSamps.next()<<12);

}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AudioOutput updateAudio(){
  int8_t asig = carrier.next(); // get this so it can be used twice without calling next() again
  int8_t delsig = aDel.next(asig, deltime); //changed from deltime
  //return asig/8 + aDel.next(asig, (uint16_t) del_samps); // mix some straight signal with the delayed signal
  //return aDel.next(aTriangle.next(), (uint16_t) del_samps); // instead of the previous 2 lines for only the delayed signal
  return MonoOutput::fromAlmostNBit(17, ((asig >> 3) + delsig)); // mix some straight signal with the delayed signal
}


void loop(){
  audioHook();
}
