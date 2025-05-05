#include <Mozzi.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <Wire.h>
#include "Adafruit_Trellis.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// === Oscillators === //
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscIndex1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscIndex2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscIndex3(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscMiddle1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscMiddle2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscMiddle3(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscRing1(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscRing2(SIN2048_DATA);
Oscil<SIN2048_NUM_CELLS, MOZZI_AUDIO_RATE> oscRing3(SIN2048_DATA);

// === Trellis === //
Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis = Adafruit_TrellisSet(&matrix0);
#define INTPIN A5
#define NUMTRELLIS 1
#define numKeys (NUMTRELLIS * 16)
int toggleArray[numKeys] = { 0 };

// === BNO055 === //
#define BNO_ENABLED true
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
sensors_event_t orientationData;
double x, y, z;

// === Pins === //
#define THUMB_PIN 4   // Volume
#define INDEX_PIN 3   // Chord set 1
#define MIDDLE_PIN 2  // Chord set 2
#define RING_PIN 1    // Chord set 3

// === Globals === //
int volume;
unsigned long previousMillis = 0, previousMillis2 = 0, currentMillis;
long delay_time_buttons = 30, delay_time_imu = 150;

void setup() {
  Serial.begin(115200);
  startMozzi();

  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);
  trellis.begin(0x70);

  if (BNO_ENABLED && !bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
}

int getChordZone(int val) {
  if (val < 1300) return 0;
  if (val < 2700) return 1;
  return 2;
}

void playChord(int chordId, int fingerId) {
  switch (fingerId) {
    case 0: // Index
      if (chordId == 0) { osc1.setFreq(261); osc2.setFreq(329); osc3.setFreq(392); } // C
      else if (chordId == 1) { osc1.setFreq(196); osc2.setFreq(247); osc3.setFreq(294); } // G
      else { osc1.setFreq(349); osc2.setFreq(440); osc3.setFreq(523); } // F
      break;
    case 1: // Middle
      if (chordId == 0) { osc1.setFreq(220); osc2.setFreq(261); osc3.setFreq(329); } // Am
      else if (chordId == 1) { osc1.setFreq(294); osc2.setFreq(349); osc3.setFreq(440); } // Dm
      else { osc1.setFreq(329); osc2.setFreq(415); osc3.setFreq(494); } // E
      break;
    case 2: // Ring
      if (chordId == 0) { osc1.setFreq(329); osc2.setFreq(392); osc3.setFreq(494); } // Em
      else if (chordId == 1) { osc1.setFreq(247); osc2.setFreq(294); osc3.setFreq(349); } // B dim
      else { osc1.setFreq(220); osc2.setFreq(277); osc3.setFreq(329); } // A
      break;
  }
}

void updateControl() {
  int thumbRaw = mozziAnalogRead<12>(THUMB_PIN);
  int indexRaw = mozziAnalogRead<12>(INDEX_PIN);
  int middleRaw = mozziAnalogRead<12>(MIDDLE_PIN);
  int ringRaw = mozziAnalogRead<12>(RING_PIN);

  volume = map(thumbRaw, 2900, 3100, 0, 255);

  playChord(oscIndex1, oscIndex2, oscIndex3, getChordZone(indexRaw), 0);
  playChord(oscMiddle1, oscMiddle2, oscMiddle3, getChordZone(middleRaw), 1);
  playChord(oscRing1, oscRing2, oscRing3, getChordZone(ringRaw), 2);

  currentMillis = millis();
  if (currentMillis - previousMillis > delay_time_buttons) {
    updateButtons();
    previousMillis = currentMillis;
  }
  if (BNO_ENABLED && currentMillis - previousMillis2 > delay_time_imu) {
    updateIMU();
    previousMillis2 = currentMillis;
  }
}

AudioOutput updateAudio() {
  int sum = oscIndex1.next() + oscIndex2.next() + oscIndex3.next()
          + oscMiddle1.next() + oscMiddle2.next() + oscMiddle3.next()
          + oscRing1.next() + oscRing2.next() + oscRing3.next();
  return MonoOutput::from16Bit(sum * volume);
}

void updateIMU() {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.type == SENSOR_TYPE_ORIENTATION) {
    x = orientationData.orientation.x;
    y = orientationData.orientation.y;
    z = orientationData.orientation.z;
    Serial.print("Orient: x="); Serial.print(x);
    Serial.print(" | y="); Serial.print(y);
    Serial.print(" | z="); Serial.println(z);
  }
}

void updateButtons() {
  if (trellis.readSwitches()) {
    for (uint8_t i = 0; i < numKeys; i++) {
      if (trellis.justPressed(i)) {
        toggleArray[i] = !toggleArray[i];
        Serial.print(i);
        Serial.println(toggleArray[i] ? " on" : " off");
      }
    }
  }
}

void loop() {
  audioHook();
}
