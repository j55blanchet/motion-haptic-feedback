#include <Arduino.h>

// https://forum.arduino.cc/t/arduino-as-spi-slave/52206/2

char buf[100];
volatile byte pos;
volatile boolean process_it;

void setup() {
  Serial.begin(9600);

  // Have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);

  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // enable interrupts
  SPCR |= _BV(SPIE);

  pos = 0;
  process_it = false;
}

ISR(SPI_STC_vect) {
  byte c = SPDR;

  if (pos < sizeof buf){
    buf[pos++] = c;
    // example: newline means time to process buffer
    if (pos >= 3){
      process_it = true;
    }
  }
}

void loop() {
  if (process_it) {

    // Add NULL terminator (unnecessary with loop, but whatever)
    buf[pos] = 0;

    // Look thorugh and print nibbles
    for(auto i = 0; i < pos; i++){
      uint8_t c = buf[i];
      char printStr[20] = {0};
      sprintf(printStr, "%02X ", c);
      Serial.print(printStr);
    }
    pos = 0;
    Serial.println();
    process_it = false;
  }
}