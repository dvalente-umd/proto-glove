/*  Example of filtering a wave,
    using Mozzi sonification library.

    Demonstrates ResonantFilter<uint8_t, FILTER_TYPE>>.
    
    Note that, on 8bits platforms (Arduino) this filter cannot work
    on samples of more than 8bits. Use LowPassFilter16() if you need
    more than that.

    Circuit: Audio output on digital pin 9 on a Uno or similar, or
    DAC/A14 on Teensy 3.1, or
    check the README or http://sensorium.github.io/Mozzi/

   Mozzi documentation/API
   https://sensorium.github.io/Mozzi/doc/html/index.html

   Mozzi help/discussion/announcements:
   https://groups.google.com/forum/#!forum/mozzi-users

   Copyright 2012-2024 Tim Barrass and the Mozzi Team

   Mozzi is licensed under the GNU Lesser General Public Licence (LGPL) Version 2.1 or later.
*/

#include <Mozzi.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h> // recorded audio wavetable
#include <tables/saw2048_int8.h>
#include <tables/cos2048_int8.h> // for filter modulation
#include <ResonantFilter.h>
#include <mozzi_rand.h>
#include<Smooth.h>

Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> carrier(SIN2048_DATA);
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kFilterMod(COS2048_DATA); //LFO 1
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kFilterMod2(COS2048_DATA); //LFO 2

//Different types of filters available
//LowPassFilter rf; // Equivalent to ResonantFilter<LOWPASS>
//ResonantFilter<HIGHPASS> rf; // HighPass filter
//ResonantFilter<BANDPASS> rf; // BandPass filter
ResonantFilter<NOTCH> rf; // Notch filter
ResonantFilter<NOTCH> rf2;

float smoothness = 0.9975f;
Smooth <long> aSmoothGain(smoothness);

uint8_t resonance = 0; // range 0-255, 255 is most resonant

byte volume;

void setup(){
  startMozzi();
  carrier.setFreq(1000);
  kFilterMod.setFreq(1.3f);
  kFilterMod2.setFreq(1.3f);
}

void loop(){
  audioHook();
}

void updateControl(){
  
  float lfo_freq1 =  (float)mozziAnalogRead<8>(0)/64;
  float lfo_freq2 = (float)mozziAnalogRead<8>(1)/64;

  volume = mozziAnalogRead<8>(2);

  kFilterMod.setFreq(lfo_freq1);
  kFilterMod2.setFreq(lfo_freq2);

  // map the modulation into the filter range (0-255), corresponds with 0-MOZZI_AUDIO_RATE/(sqrt(2)*pi) Hz
  byte cutoff_freq = 100 + kFilterMod.next()/2;
  byte cutoff_freq2 = 100 + kFilterMod2.next()/2;
  rf.setCutoffFreqAndResonance(cutoff_freq, resonance);
  rf2.setCutoffFreqAndResonance(cutoff_freq2, resonance);
}

AudioOutput updateAudio(){
  char filtsig = rf2.next((char)rf.next(carrier.next()));
  return MonoOutput::fromNBit(16, (volume*(filtsig)));
}
