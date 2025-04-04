#include <Wire.h>
#include "Adafruit_Trellis.h"


Adafruit_Trellis matrix0 = Adafruit_Trellis();
Adafruit_TrellisSet trellis = Adafruit_TrellisSet(&matrix0);


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


void setup() {
  Serial.begin(9600);
  Serial.println("Trellis Demo");

  // INT pin requires a pullup
  pinMode(INTPIN, INPUT);
  digitalWrite(INTPIN, HIGH);

  // begin() with the addresses of each panel in order
  // I find it easiest if the addresses are in order
  trellis.begin(0x70);
}

void loop() {
  delay(30);  // 30ms delay is required, dont remove me!

  // If a button was just pressed or released...
  if (trellis.readSwitches()) {
    // go through every button
    for (uint8_t i = 0; i < numKeys; i++) {
      // if it was pressed...
      if (trellis.justPressed(i)) {
        Serial.print("v");
        Serial.println(i);
        // Alternate the LED
        toggleArray[i] = !toggleArray[i];
        Serial.print(toggleArray[i]);
        Serial.println(i);
        // tell the trellis to set the LEDs we requested
        trellis.writeDisplay();
      }
    }
  }
}
