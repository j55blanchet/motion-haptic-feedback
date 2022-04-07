#include <Arduino.h>
#include <SPI.h>
#include "bos1901.h"
#include "utils.h"

const std::vector<int> csPins = {D0, D1, D2, D3};
std::vector<BOS1901*> chips;
bool hasPrinted = false;


void countdown(unsigned int count) {
  while (count > 0) {
    Serial.print(count);
    Serial.print("...");
    delay(1000);
    count--;
  }
}


void writeWaveformPulse(BOS1901& chip) {
  const auto TARGET_HZ = 250U; // the freq. at which fingertips are most sensitive
  const auto SAMPLE_RATE = 8000U; // 8ksps - the freq at which we operate the device
  const auto SAMPLES_TOTAL = SAMPLE_RATE / TARGET_HZ;

  // 
  for(auto i = 0U; i < SAMPLES_TOTAL / 2; i++) {
    chip.writeFifoVoltage(chip.maxPiezoVoltage);
  }
  for(auto i = 0U; i < SAMPLES_TOTAL / 2; i++) {
    chip.writeFifoVoltage(chip.minPiezoVoltage);
  }
  chip.writeFifoVoltage(0);
}

void writeSinWave(BOS1901& chip, int repititions = 1, int hz=250, int sampleRate=8000) {
  const auto SAMPLES_PER_CYCLE = sampleRate / hz;
  const auto SAMPELS_TOTAL = SAMPLES_PER_CYCLE * repititions;

  const auto offset = (chip.maxPiezoVoltage + chip.minPiezoVoltage) / 2;
  const auto amplitude = (chip.maxPiezoVoltage - chip.minPiezoVoltage) / 2;

  for(auto i = 0; i < SAMPELS_TOTAL; i++) {
    const double t = (i % SAMPLES_PER_CYCLE) / (double)SAMPLES_PER_CYCLE;
    const double v = offset + amplitude * sin(2 * TWO_PI * t);
    chip.writeFifoVoltage(v);
  }
  chip.writeFifoVoltage(0.0);
}

void setup() { 

  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("\n\nBOS1901 SPI test");

  for(std::size_t i = 0; i < std::size(csPins); i++) {
    Serial.println(String("Initializing chip ") + i);
    chips.push_back(new BOS1901(
      SPI,
      csPins[i],
      BOS1901::PLAYBACK_SPEED_8_ksps,
      true,  // lock registers when not writing
      5.0,   // VBUS=5V
      -10.0, // Min piezo voltage
      60.0   // Max piezo voltage
    ));
  }

  // auto offset = chip->getADCoffset();
  // chip -> setSquareWavePlayback(false);
  // Serial.println("ADC offset: " + String(offset));

  for(std::size_t i = 0; i < std::size(chips); i++) {
    const auto chip = chips[i];

    Serial.println("Scanning registers for chip " + String(i) + ":");
    chip->scanRegisters(true);
    Serial.println();
  }

  Serial.println("Enabling Output & Square Wave Playback");
  for (std::size_t i = 0; i < std::size(chips); i++) {

    chips[i]->setOutputEnabled(true);
    chips[i]->setSquareWavePlayback(true);
  }
}

String inputBuffer = "";

void loop() {

  if (!hasPrinted) {
    hasPrinted = true;
    Serial.println("\nStarting CMD Line Mode\n");
  }

  Serial.println("Type a command (pulse, sense, sin):");

  String command = "";
  while (command.isEmpty()) {
    while (!Serial.available()) {
      delay(1);
    }

    String receivedStr = Serial.readString();
    inputBuffer.concat(receivedStr);
    Serial.println(inputBuffer);
    auto newlineIndex = inputBuffer.indexOf("\n");
    if (newlineIndex != -1) {
      command = inputBuffer.substring(0, newlineIndex);
      inputBuffer = inputBuffer.substring(newlineIndex + 1);
    }
  }

  Serial.println("GOT COMMAND: " + command);
  std::vector<String> args = {};
  split(command, ' ', args);

  if (std::size(args) == 0) {
    return;
  }
  args[0].toLowerCase();
  command = args[0];

  auto bIndex = std::size(args) > 1 ? args[1].toInt() : 0;
  if ((unsigned long)bIndex >= std::size(chips)) {
    Serial.println("Invalid board number");
    return;
  }
  std::vector<long> boardIndices = {bIndex};
  if (args[1].startsWith("all")) {
    Serial.println("Using all boards");
    boardIndices.clear();
    for (std::size_t i = 0; i < std::size(chips); i++) {
      boardIndices.push_back(i);
    }
  }

  bool doCountDown = true;
  for (auto boardIndex : boardIndices) {
    Serial.println("Board " + String(boardIndex));
    if (command.startsWith("sense")) {
      Serial.println("  Will capture sensed voltage for a few secs....");
      if (doCountDown) {
        countdown(3);
        Serial.println();
        doCountDown = false;
      }

      chips[boardIndex]->setSENSE(false);
      writeWaveformPulse(*chips[boardIndex]);
      delay(50);

      Serial.println();
      auto voltageRecordings = std::vector<double>();
      
      // wait for positive polarity
      auto timeStart = millis();
      chips[boardIndex]->initializeSensing(true, true);
      while(millis() - timeStart < 1000) {
        voltageRecordings.push_back(chips[boardIndex]->senseVoltage());
        delay(1);
      }

      chips[boardIndex]->setSENSE(false);
      writeWaveformPulse(*chips[boardIndex]);
      delay(20);
      writeWaveformPulse(*chips[boardIndex]);
      delay(20);
      writeWaveformPulse(*chips[boardIndex]);

      Serial.println("Complete. Measurements:");
      for (std::size_t i = 0; i < std::size(voltageRecordings); i++) {
        Serial.println(voltageRecordings[i]);
      }
      Serial.println();
      writeWaveformPulse(*chips[boardIndex]);
      delay(20);
    }

    if (command.startsWith("pulse")) {
      Serial.println("  Will pulse 5 times....");
      if (doCountDown) {
        countdown(3);
        Serial.println();
        doCountDown = false;
      }

      for(auto i = 0; i < 5; i++) {
        writeWaveformPulse(*chips[boardIndex]);
        delay(50);
      }
    }

    if (command.startsWith("sin")) {
      Serial.println("  Will output sin wave with ramping freq...");
      if (doCountDown) {
        countdown(3);
        Serial.println();
        doCountDown = false;
      }

      for(auto i = 10; i < 200; i++) {
        writeSinWave(*chips[boardIndex], 1, i);
        Serial.print(String(i) + " ");
      }
      Serial.println();
    }

    if (command.startsWith("button")) {
      Serial.println("  Will do button IO...");
      if (doCountDown) {
        countdown(3);
        Serial.println();
        doCountDown = false;
      }

      auto v_thres = 0.5;
      
    }
  }
}
