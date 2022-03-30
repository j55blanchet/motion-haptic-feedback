#include <Arduino.h>
#include <SPI.h>
#include "bos1901.h"


BOS1901 *chip = NULL;

void setup() { 

  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("\nBOS1901 SPI test");

  chip = new BOS1901(
    SPI, 
    D1, // Chip select PIN
    BOS1901::PLAYBACK_SPEED_8_ksps, 
    true,  // Lock registers when OE = true
    5.0,   // VBUS = 5V
    -10.0, // Min piezo voltage
    60);   // Max piezo voltage = 60V

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

  const double secs = 7;
  const int sample_rate = 60;
  bool gotPress = false;
  const float voltage_threshold = 5.5;
  for(auto i = 0; i < sample_rate * secs; i++) {
    auto voltage = chip -> senseVoltage();

    if (i % sample_rate == 0 || voltage > voltage_threshold) {
      chip -> print_reg_sense(false);
    }
  
    if (voltage > voltage_threshold) {
      Serial.println("Detected button press! Sensed voltage: " + String(voltage) + "V");
      gotPress = true;
      break;
    }
    // auto voltage = chip->senseVoltage();
    // Serial.print("Voltage: ");
    // Serial.print(String(i) + "/" + String(i_max) + ": ");
    // Serial.println(voltage);
    delay(1000.0 / sample_rate);
  }

  if (gotPress) {
    const float Hz = 100;
    const int sample_rate = 8000;
    const int samples_per_cycle = sample_rate / Hz;

    const int cycles = 10; // 1 sec
    int sample_id = 0;

    for (int i = 0; i < cycles; i ++) {
      for (float t = 0; t < PI; t += PI / samples_per_cycle) {
        const float multiplier = sin(t);
        const float voltage = multiplier * (multiplier > 0 ? chip->maxPiezoVoltage : chip->minPiezoVoltage);
        while(!chip -> hasFifoSpace()) {
          delayMicroseconds(16);
        }
        chip -> writeToFifo(voltage);
        sample_id ++;
        // Serial.println("[" + String(sample_id) + "]: " + String(voltage) + "V");
      }
      // chip -> print_reg_ic_status(false);
    }

    // Ensure we end on zero  
    chip -> writeToFifo(0);
  }

  Serial.println();
}

// 11110000000
// 11110000000
// 111110100111
// 1111 1 0 1 0 111