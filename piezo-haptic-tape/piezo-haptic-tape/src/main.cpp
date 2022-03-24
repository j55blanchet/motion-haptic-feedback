#include <Arduino.h>
#include <SPI.h>
#include "bos1901.h"


BOS1901 *chip = NULL;

void setup() {


  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);

  chip = new BOS1901(SPI, D8);

  auto offset = chip->getADCoffset();
  Serial.println("ADC offset: " + String(offset));
}

void loop() {
  // put your main code here, to run repeatedly:

  // // Serial.println("Looping!");
  // digitalWrite(LED_BUILTIN, HIGH);
  // // Serial.println("High");
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);
  // // Serial.println("Low");

  auto voltage = chip->senseVoltage();
  Serial.println("Voltage: " + String(voltage));
  delay(300);
}

// 11110000000
// 11110000000
// 111110100111
// 1111 1 0 1 0 111