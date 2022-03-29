#include <Arduino.h>
#include <SPI.h>
#include "bos1901.h"


BOS1901 *chip = NULL;

void setup() {


  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("\nBOS1901 SPI test");

  chip = new BOS1901(SPI, D1, BOS1901::PLAYBACK_SPEED_8_ksps, true, 5.0);

  auto offset = chip->getADCoffset();
  Serial.println("ADC offset: " + String(offset));

  Serial.println("Scanning Registers");
  chip->scanRegisters(true);
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:

  // // Serial.println("Looping!");
  // digitalWrite(LED_BUILTIN, HIGH);
  // // Serial.println("High");
  // delay(1000);
  // digitalWrite(LED_BUILTIN, LOW);
  // // Serial.println("Low");

  Serial.println("Scanning Registers");
  chip->scanRegisters(false);
  Serial.println();

  // auto voltage = chip->senseVoltage();
  // Serial.println("Voltage: " + String(voltage));
  // delay(3000);

  const auto i_max = 20;
  for(auto i = 0; i < i_max; i++) {
    chip -> print_reg_sense(false);
    // auto voltage = chip->senseVoltage();
    // Serial.print("Voltage: ");
    // Serial.print(String(i) + "/" + String(i_max) + ": ");
    // Serial.println(voltage);
    delay(500);
  }

  Serial.println();
}

// 11110000000
// 11110000000
// 111110100111
// 1111 1 0 1 0 111