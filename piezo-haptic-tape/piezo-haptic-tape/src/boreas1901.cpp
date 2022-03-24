#include <Arduino.h>
#include <SPI.h>

#include "boreas1901.h"

//// ARDUINO SPI MODES TABLE ////
// From: https://www.arduino.cc/en/reference/SPI
//
// Mode	        Clock Polarity (CPOL)	Clock Phase (CPHA)	Output Edge	    Data Capture
// SPI_MODE0	0	                    0	                Falling	        Rising
// SPI_MODE1	0	                    1	                Rising	        Falling
// SPI_MODE2	1	                    0	                Rising	        Falling
// SPI_MODE3	1 	                    1	                Falling	        Rising
//
// The BOS1901 chip SPI coniguration is:
// * Input data is transitioned on the falling edge of SCL. (output edge = falling)
// * Data is latched on the rising edge of SCL. (data capture = rising)
// * The most significant bit is sent first.
// * Data rates of up to 35 Mbps are supported.
// * CS should be set LOW to enable the chip.

BOS1901::BOS1901(SPIClass &spi, uint8_t chipSelectPin): _spi(spi), _chipSelectPin(chipSelectPin) {
}
    

uint16_t BOS1901::makeCommand(uint8_t reg_addr, uint16_t data) {
    uint16_t command = (reg_addr) << 12 + data & 0x0FFF;
    return command;
}

uint16_t BOS1901::transfer(uint16_t write_command) {
    const uint32_t clock = 2000000; // 2 MHz. Max is 35 MHz.
    const uint8_t spi_mode = SPI_MODE0;
    _spi.beginTransaction(SPISettings(clock, MSBFIRST, spi_mode));

    digitalWrite(_chipSelectPin, LOW);
    uint16_t receivedData = _spi.transfer16(write_command);
    digitalWrite(_chipSelectPin, HIGH);

    _spi.endTransaction();
    return receivedData;
}

void BOS1901::setConfig(uint8_t output_reg_select, bool lock_registers, bool enable_waveform_playback, uint8_t playback_speed) {
    uint16_t config = 0x0000;
    config |= (output_reg_select << 7);
    config |= lock_registers << 6;
    config |= enable_waveform_playback << 4;
    config |= playback_speed & 0b00000111;

    uint16_t command = makeCommand(REG_CONFIG, config);
    transfer(command);
}

void BOS1901::setSENSE(bool enableSensing) {
    uint16_t command_data = enableSensing << 11; // Enable SENSE
    uint16_t command = makeCommand(REG_SUP_RISE, command_data);
    transfer(command);
}

void BOS1901::reset() {
    uint8_t reset_bit = 0x01 << 5;
    uint16_t command = makeCommand(REG_CONFIG, reset_bit);
    transfer(command);
}

uint16_t BOS1901::writeWaveform(uint16_t waveForm) {
    uint16_t sanitizedWaveform = waveForm & 0x0FFF;
    uint16_t command = makeCommand(REG_REFERENCE, sanitizedWaveform);
    return transfer(command);
}

uint16_t BOS1901::senseVoltage() {

    // Set chip to SENSE, which mutes the waveform output, allowing 
    // the voltage to move freely and for us to get a reading.
    setSENSE(true);

    // Set output to the sensing register (ensure OE is enabled)
    setConfig(REG_SENSE, false, true, 0);
    uint16_t command = makeCommand(REG_SENSE, 0x0000);
    _spi.write16(command);

    // Read the output
    return transfer(command);
}

uint16_t BOS1901::getADCoffset() {

    // Set output to the sensing register (ensure OE is enabled)
    setConfig(REG_SENSE, false, true, 0);
    uint16_t command = makeCommand(REG_SENSE, 0x0000);
    _spi.write16(command);

    // Set sense to false, allowing us to drive output to 0V
    setSENSE(false);
    writeWaveform(0x0000);

    uint16_t offsetResult = writeWaveform(0x0000);
    offsetResult &= 0x0CFF; // Get VFEEDBACK (lowest 10 bits).

    return offsetResult;
}
