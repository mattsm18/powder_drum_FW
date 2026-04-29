
/*
Title: main.cpp
Author: Matthew Smith
Date: 29/04/26
Purpose:
-  Entry point for powder drum FW
-  Initialise pins
-  Handle primary function calls for all components in the FW
*/

#include <Arduino.h>
#include "pins.h"

void setup() {
  
  // TB6600 Pins
  pinMode(STEPPER_DIR_PIN, OUTPUT);
  pinMode(STEPPER_PUL_PIN, OUTPUT);

  // 5V Relay module
  pinMode(LED_PANEL_RELAY_PIN, OUTPUT);

  // I2C comms setup (AS5600 encoder)


}

void loop() {
  // put your main code here, to run repeatedly:
}
