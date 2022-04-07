#include <Arduino.h>
#include <SPI.h>

#include "bos1901.h"

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

// Helper function
int16_t UpscaleTwosComplement(int16_t value, size_t length)
{
    // Too large?
    if (length > 15)
    {
        Serial.println("UpscaleTwosComplement length too large");
        return -1;
    }
    uint16_t mask = (~0) << length;
    // Too small for complement?
    if (length < 2)
    {
        return (~mask & value);
    }
    // Two's complement?
    uint16_t msb = 1 << (length-1);
    if (value & msb)
    {
        return (mask | value);
    }
    else
    {
        return (~mask & value);
    }
}

float computeSensedVoltage(uint16_t v_feedback) {
    v_feedback = v_feedback & 0x3FF;
    return (v_feedback) * 3.6f * 31 / (pow(2,10) - 1);
};

const uint32_t bos1901_spi_freq = 200000;
const uint8_t bos1901_spi_mode = SPI_MODE0;

BOS1901::BOS1901(
    SPIClass &spi, 
    uint8_t chipSelectPin, 
    uint8_t playbackSpeed, 
    bool lock_registers,
    float supply_voltage,
    float minPiezoVoltage,
    float maxPiezoVoltage
    ): 
        _spi(spi), 
        _chipSelectPin(chipSelectPin), 
        _playbackSpeed(playbackSpeed), 
        _lockRegisters(lock_registers),
        minPiezoVoltage(minPiezoVoltage),
        maxPiezoVoltage(maxPiezoVoltage)
{
    _outputEnabled = false;
    _outputRegister = REG_DEADTIME;
    _lastReceivedDataSource = REG_DEADTIME;
    _lastReceivedData = 0x0000;
    _outputMutedForSensing = false;

    pinMode(_chipSelectPin, OUTPUT);
    // _spi.setFrequency(bos1901_spi_freq);
    // _spi.setDataMode(bos1901_spi_mode);
    _spi.begin();

    delay(1000);

    // Unlock registers
    setConfig(REG_DEADTIME, false, true);

    int32_t vdd = (int32_t) (((supply_voltage / 0.026) - 128) / 2);
    uint8_t vdd_config = (uint8_t) vdd;
    if (vdd > 31) vdd_config = 0x1F;
    else if (vdd < 0) vdd_config = 0x00;
    _vddSupRise = vdd_config;

    uint16_t sup_rise_data = 0x0027; // keep default for TI_RISE
    sup_rise_data |= (vdd_config << 6);
    uint16_t command = makeCommand(REG_SUP_RISE, sup_rise_data);
    Serial.println("Setting VDD in SUPRISE to " + String(vdd_config, HEX));
    transfer(command);

    // if (supply_voltage - 5.0 < 1.0) {
    //    // Disable charge pump
    //    uint16_t parcap_command = 0b0110010000111010; // keeps defaults, sets address to parcap register
    //    transfer(parcap_command);
    //}   
}

uint16_t BOS1901::makeCommand(uint8_t reg_addr, uint16_t data) {
    uint16_t command = (reg_addr << 12) + (data & 0x0FFF);
    return command;
}

uint16_t BOS1901::transfer(uint16_t write_command) {
    _spi.beginTransaction(SPISettings(bos1901_spi_freq, MSBFIRST, bos1901_spi_mode));

    digitalWrite(_chipSelectPin, LOW);
    // uint8_t data1 = _spi.transfer(highByte(write_command));
    // uint8_t data2 = _spi.transfer(lowByte(write_command));


    _lastReceivedData = _spi.transfer16(write_command);
    _lastReceivedDataSource = _outputRegister;
    digitalWrite(_chipSelectPin, HIGH);

    _spi.endTransaction();

    // uint16_t receivedData = (data1 << 8) | data2;

    // Serial.print("  SPI Transfer. SENT: ");
    // Serial.print(write_command, HEX);
    // Serial.print(" RECEIVED: ");
    // Serial.println(receivedData, HEX);
    return _lastReceivedData;
}

void BOS1901::setOutputEnabled(bool enabled) {
    _outputEnabled = enabled;
    setConfig(_outputRegister, enabled);
}

uint16_t BOS1901::setConfig(uint8_t output_reg_select, bool enable_waveform_playback, bool override_unlock_registers) {
    _outputEnabled = enable_waveform_playback;
    _outputRegister = output_reg_select;

    uint16_t config = 0x0000;
    config |= (output_reg_select << 7);
    config |= ((!override_unlock_registers) && _lockRegisters) << 6;
    config |= enable_waveform_playback << 4;
    config |= _playbackSpeed & 0b00000111;

    uint16_t command = makeCommand(REG_CONFIG, config);
    // Serial.println("Setting config. New output register: 0x" + String(output_reg_select, HEX) + "  Command: 0x" + String(command, HEX) + "  Config: 0x" + String(config, HEX));
    return transfer(command);
}

void BOS1901::initializeSensing(bool muteWaveformOutput, bool positivePolarity) {
    // Refresh ADC offset - first step: Drive to 0
    setConfig(REG_SENSE, true, true);
    setSENSE(false);
    writeToFifo(0.0);
    
    _feedbackAdcOffset = getRegisterContents(
        REG_SENSE, 
        false,     // don't allow cached value
        REG_SENSE, // likely to use REG_SENSE for sensing after this
        true       // keep registers unlocked
    );

    // Now, mute output if desired
    if (muteWaveformOutput) {
        setSENSE(muteWaveformOutput);
    }
    setConfig(REG_SENSE, true, false);
    writeToFifo(positivePolarity ? 0x000 : 0xFFF); // set H-bridge in positive polarity
}

void BOS1901::setSENSE(bool enableSensing) {
    
    if (_outputMutedForSensing == enableSensing) {
        return;
    }

    uint16_t command_data = 0x0000; // Enable SENSE
    command_data |= enableSensing << 11;
    command_data |= _vddSupRise << 6;

    uint8_t ti_rise_default = 0x27;
    command_data |= ti_rise_default;

    uint16_t command = makeCommand(REG_SUP_RISE, command_data);
    transfer(command);
    _outputMutedForSensing = enableSensing;
}

uint16_t BOS1901::getRegisterContents(uint8_t reg_addr, bool allow_cached, uint8_t next_reg_addr, bool keep_registers_unlocked) {
    if (reg_addr != _outputRegister) {
        setConfig(reg_addr, _outputEnabled, keep_registers_unlocked);
    } else if (allow_cached && _lastReceivedDataSource == reg_addr) {
        return _lastReceivedData;
    }
    
    auto next_reg = (next_reg_addr == REG_NONE) ? reg_addr : next_reg_addr;
    return setConfig(next_reg, _outputEnabled, keep_registers_unlocked);
}

uint16_t BOS1901::getContentOfResponse(uint16_t data) {
    return data & 0x0FFF;
}

uint8_t BOS1901::getRegisterOfResponse(uint16_t data) {
    return data >> 12;
}

void BOS1901::print_reg_reference(bool verbose) {
    uint16_t fifo = getRegisterContents(REG_REFERENCE);
    uint16_t fifo_content = getContentOfResponse(fifo);
    int16_t fifo_dec = UpscaleTwosComplement((int16_t) fifo_content, 12);

    if (verbose) {
        Serial.println("REG_REFERENCE: (0x" + String(fifo, HEX) + "):");
        Serial.println("  FIFO: " + String(fifo_dec));
    } else {
        Serial.println("REG_REFERENCE (0x" + String(fifo, HEX) + "): " + String(fifo_dec));
    }
}


void BOS1901::writeToFifo(uint16_t value) {
    uint16_t command = makeCommand(REG_REFERENCE, value & 0x0FFF);
    transfer(command);
}
void BOS1901::writeFifoVoltage(float voltage) {

    voltage = min(voltage, maxPiezoVoltage);
    voltage = max(voltage, minPiezoVoltage);

    const float vref = 3.6f;
    const float fb_ratio = 31.0f;
    const float divider = vref * fb_ratio;
    const float multiplier = pow(2, 11) - 1.0f;
    float amplitude_signal = voltage * multiplier / divider;
    int16_t int_amplitude = static_cast<int16_t>(amplitude_signal);
    int_amplitude = min(int_amplitude, (int16_t) 1743);
    int_amplitude = max(int_amplitude, (int16_t) -1742);

    uint16_t data = highByte(int_amplitude) << 8;
    data |= lowByte(int_amplitude);
    
    writeToFifo(data);
}

void BOS1901::setSquareWavePlayback(bool enable) {

    // Unlock registers
    setConfig(_outputRegister, _outputEnabled, true);

    // Set square wave enabled bit on KP register
    uint16_t kp_val = 0x080;
    uint16_t kp_command_data = (enable << 11) | kp_val;
    uint16_t command = makeCommand(REG_KP, kp_command_data);
    transfer(command);

    // Relock registers
    setConfig(_outputRegister, _outputEnabled, false);
}

void BOS1901::print_reg_ion_bl(bool verbose) {
    ////// ION_BLOCK
    auto ion_bl = getRegisterContents(REG_ION_BL);
    auto ion_bl_contents = getContentOfResponse(ion_bl);

    // Boost converter maximum switching frequency
    uint8_t fswmax = (ion_bl_contents >> 10) & 0x3; 
    String fswmax_hz = "Unknown";
    if (fswmax == 0) fswmax_hz = "1 MHz-default";
    else if (fswmax == 1) fswmax_hz = "833 kHz";
    else if (fswmax == 2) fswmax_hz = "666 kHz";
    else if (fswmax == 3) fswmax_hz = "500 kHz";

    // Boost converter blanking time (default: 0x3)
    uint8_t sb = (ion_bl_contents & 0x0300) >> 8;
    String sb_ns = "Unknown";
    if (sb == 0x0) sb_ns = "35ns";
    else if (sb == 0x1) sb_ns = "44ns";
    else if (sb == 0x2) sb_ns = "53ns";
    else if (sb == 0x3) sb_ns = "62ns-default";
    
    // Minimum current required to turn ON HS. (default: 0xA0)
    uint8_t ionscale = ion_bl_contents & 0xFF;
    String ionscale_str = String(ionscale);
    if (ionscale == 0xA0) ionscale_str += "-default";

    if (verbose) {
        Serial.println("ION_BLOCK (0x" + String(ion_bl, HEX) + "):");
        Serial.println("  fswmax: 0x" + String(fswmax, HEX) + " (" + fswmax_hz + ")");
        Serial.println("  sb: 0x" + String(sb, HEX) + " (" + String(sb_ns) + ")");
        Serial.println("  ionscale: " + ionscale_str);
    }
    else {
        Serial.print("ION_BLOCK (0x" + String(ion_bl, HEX) + "): ");
        Serial.println("0x" + String(fswmax, HEX) + "  0x" + String(sb, HEX) + "  " + ionscale_str);
    }
}

void BOS1901::print_reg_deadtime(bool verbose) {
    auto dead_time = getRegisterContents(REG_DEADTIME);
    auto dead_time_contents = getContentOfResponse(dead_time);

    auto dhs = dead_time_contents >> 5;
    String dhs_str = String(dhs);
    if (dhs == 0x23) dhs_str += "-default";

    auto dls = dead_time_contents & 0x1F;
    String dls_str = String(dls);
    if (dls == 0x0A) dls_str += "-default";

    if (verbose) {
        Serial.println("DEADTIME (0x" + String(dead_time, HEX) + "):");
        Serial.println("  dhs: " + dhs_str);
        Serial.println("  dls: " + dls_str);
    } else {
        Serial.print("DEADTIME (0x" + String(dead_time, HEX) + "): ");
        Serial.println(dhs_str + "  " + dls_str);
    }
}

void BOS1901::print_reg_kp(bool verbose) {
    auto kp_data = getRegisterContents(REG_KP);
    auto kp_contents = getContentOfResponse(kp_data);

    auto sq = kp_contents >> 11;
    String sq_str = sq ? "Square waveforms (0x1)" : "Continuous waveforms (0x0)-default";

    auto kp = kp_contents & 0x7FF;
    String kp_str = String(kp);
    if (kp == 0x080) kp_str += "-default";

    if (verbose) {
        Serial.println("KP (0x" + String(kp, HEX) + "):");
        Serial.println("  sq: " + sq_str);
        Serial.println("  kp: " + kp_str);
    } else {
        Serial.print("KP (0x" + String(kp, HEX) + "): ");
        Serial.println(sq_str + "  " + kp_str);
    }
}

void BOS1901::print_reg_kpa_ki(bool verbose) {
    auto kpa_ki = getRegisterContents(REG_KPA_KI);
    auto kpa_ki_contents = getContentOfResponse(kpa_ki);

    auto kibase = kpa_ki_contents >> 8;
    String kkibase_str = String(kibase);
    if (kibase == 0x2) kkibase_str += "-default";

    auto kpa = kpa_ki_contents & 0xFF;
    String kpa_str = String(kpa);
    if (kpa == 0xA0) kpa_str += "-default";

    if (verbose) {
        Serial.println("KPA_KI (0x" + String(kpa_ki, HEX) + "):");
        Serial.println("  kibase: " + kkibase_str);
        Serial.println("  kpa: " + kpa_str);
    } else {
        Serial.print("KPA_KI (0x" + String(kpa_ki, HEX) + "): ");
        Serial.println(kkibase_str + "  " + kpa_str);
    }
}

void BOS1901::print_reg_config(bool verbose) {
    auto config = getRegisterContents(REG_CONFIG);
    auto config_contents = getContentOfResponse(config);

    auto bc = config_contents >> 7;
    String bc_str = "0x" + String(bc, HEX);
    if (bc == 0x2) bc_str += "-default";

    bool reg_lock = (config_contents >> 6) & 0x1;
    String reg_lock_str = reg_lock ? "Registers locked (0x1)" : "Registers unlocked (0x0)-default";
    String reg_lock_concise_str = reg_lock ? "RegLocked" : "RegUnlocked";

    bool rst = (config_contents >> 5) & 0x1;
    String rst_str = rst ? "Reset (0x1)" : "Normal operation (0x0)-default";
    String rst_concise_str = rst ? "Reset" : "Noreset";

    bool oe = (config_contents >> 4) & 0x1;
    String oe_str = oe ? "Waveform playback enabled (0x1)" : "Waveform playback disabled (0x0)-default";
    String oe_concise_str = oe ? "OutputEnabled" : "OutputDisabled";

    bool ds = (config_contents >> 3) & 0x1;
    String ds_str = ds ? "SLEEP mode when not playing waveforms (0x1)" : "IDLE mode when not playing waveforms (0x0)-default";
    String ds_concise_str = ds ? "Sleep" : "Idle";

    auto playback_spd = config_contents & 0x7;
    String playback_spd_str = "0x" + String(playback_spd, HEX);
    if (playback_spd == 0x0)      playback_spd_str += "-1024ksps-default";
    else if (playback_spd == 0x1) playback_spd_str += "-512ksps";
    else if (playback_spd == 0x2) playback_spd_str += "-256ksps";
    else if (playback_spd == 0x3) playback_spd_str += "-128ksps";
    else if (playback_spd == 0x4) playback_spd_str += "-64ksps";
    else if (playback_spd == 0x5) playback_spd_str += "-32ksps";
    else if (playback_spd == 0x6) playback_spd_str += "-16ksps";
    else if (playback_spd == 0x7) playback_spd_str += "-8ksps";
    
    if (verbose) {
        Serial.println("CONFIG (0x" + String(config, HEX) + "):");
        Serial.println("  bc: " + bc_str);
        Serial.println("  reg_lock: " + reg_lock_str);
        Serial.println("  rst: " + rst_str);
        Serial.println("  oe: " + oe_str);
        Serial.println("  ds: " + ds_str);
        Serial.println("  playback_spd: " + playback_spd_str);
    } else {
        Serial.print("CONFIG (0x" + String(config, HEX) + "): ");
        Serial.println(bc_str + "  " + reg_lock_concise_str + "  " + rst_concise_str + "  " + oe_concise_str + "  " + ds_concise_str + "  " + playback_spd_str);
    }
}

void BOS1901::print_reg_parcap(bool verbose) {
    auto parcap = getRegisterContents(REG_PARCAP);
    auto parcap_contents = getContentOfResponse(parcap);

    bool upi = (parcap_contents >> 11) & 0x1;
    String upi_str = upi ? "Unidirectional power inpnut enabled (0x1)" : "Unidirectional power disabled (0x0)-default";
    String upi_str_concise = upi ? "UPI-Enabled" : "UPI-Disabled";

    bool lmi = (parcap_contents >> 10) & 0x1;
    String lmi_str = lmi ? "HS switch curret limit overriden - default (0x1)" : "HS switch curret limit not overriden (0x0)";
    String lmi_str_concise = lmi ? "LMI-Enabled-default" : "LMI-Disabled";

    bool cp5 = (parcap_contents >> 9) & 0x1;
    String cp5_str = cp5 ? "5V charge pump ON (0x1)-default" : "5V charge pump OFF (0x0)";
    String cp5_str_concise = cp5 ? "5Vpump-Enabled-default" : "5Vpump-Disabled";

    bool cal = (parcap_contents >> 8) & 0x1;
    String cal_str = cal ? "Calibration enabled (0x1)-default" : "Calibration disabled (0x0)";
    String cal_str_concise = cal ? "Calibration-Enabled-default" : "Calibration-Disabled";

    uint8_t parcap_val = parcap_contents & 0xFF;
    String parcap_val_str = String(parcap_val);
    if (parcap_val == 0x3A) parcap_val_str += "-default";

    if (verbose) {
        Serial.println("PARCAP (0x" + String(parcap, HEX) + "):");
        Serial.println("  upi: " + upi_str);
        Serial.println("  lmi: " + lmi_str);
        Serial.println("  cp5: " + cp5_str);
        Serial.println("  cal: " + cal_str);
        Serial.println("  parcap_val: " + parcap_val_str);
    } else {
        Serial.print("PARCAP (0x" + String(parcap, HEX) + "): ");
        Serial.println(upi_str_concise + "  " + lmi_str_concise + "  " + cp5_str_concise + "  " + cal_str_concise + "  " + parcap_val_str);
    }
}

void BOS1901::print_reg_sup_rise(bool verbose) {
    auto sup_rise = getRegisterContents(REG_SUP_RISE);
    auto sup_rise_contents = getContentOfResponse(sup_rise);

    bool sense = (sup_rise_contents >> 11) & 0x1;
    String sense_str = sense ? "Sense enabled, output muted (0x1)" : "Sense disabled (0x0)-default";
    String sense_str_concise = sense ? "Sense-Enabled" : "Sense-Disabled-default";

    uint8_t vdd = (sup_rise_contents >> 6) & 0x1f;
    float vdd_v = (((float) vdd * 2.0) + 128.0) * 0.026;
    String vdd_str = "0x" + String(vdd, HEX) + "-" + String(vdd_v, 2) + "V";
    if (vdd == 0x05) vdd_str += "-default"; 

    uint8_t tirise = sup_rise_contents & 0x3F;
    String tirise_str = String(tirise);
    if (tirise == 0x27) tirise_str += "-default";

    if (verbose) {
        Serial.println("SUP_RISE (0x" + String(sup_rise, HEX) + "):");
        Serial.println("  sense: " + sense_str);
        Serial.println("  vdd: " + vdd_str);
        Serial.println("  tirise: " + tirise_str);
    } else {
        Serial.print("SUP_RISE (0x" + String(sup_rise, HEX) + "): ");
        Serial.println(sense_str_concise + "  " + vdd_str + "  " + tirise_str);
    }
}

void BOS1901::print_reg_dac(bool verbose) {
    auto dac = getRegisterContents(REG_DAC);
    auto dac_contents = getContentOfResponse(dac);

    uint8_t dac_hs = (dac_contents >> 6) & 0x3F;
    String dac_hs_str = String(dac_hs);
    if (dac_hs == 0x2) dac_hs_str += "-default";

    uint8_t dac_ls = dac_contents & 0x3F;
    String dac_ls_str = String(dac_ls);
    if (dac_ls == 0x2) dac_ls_str += "-default";

    if (verbose) {
        Serial.println("DAC (0x" + String(dac, HEX) + "):");
        Serial.println("  dac_hs: " + dac_hs_str);
        Serial.println("  dac_ls: " + dac_ls_str);
    } else {
        Serial.print("DAC (0x" + String(dac, HEX) + "): ");
        Serial.println(dac_hs_str + "  " + dac_ls_str);
    }
}

void BOS1901::print_reg_ic_status(bool verbose) {
    auto ic_status = getRegisterContents(REG_IC_STATUS);
    auto ic_status_contents = getContentOfResponse(ic_status);

    uint8_t state = (ic_status_contents >> 10) & 0x3;
    String state_str = "Unknown";
    if (state == 0x0) state_str = "IDLE";
    else if (state == 0x1) state_str = "CALIBRATION";
    else if (state == 0x2) state_str = "RUN";
    else if (state == 0x3) state_str = "ERROR";

    bool ovv = (ic_status_contents >> 9) & 0x1;
    String ovv_str = ovv ? "Output-Voltage-OVERVOLTED!" : "Output-Voltage-OK";

    bool ovt = (ic_status_contents >> 8) & 0x1;
    String ovt_str = ovt ? "Output-Temperature-EXCEEDED!" : "Output-Temperature-OK";

    bool full = (ic_status_contents >> 7) & 0x1;
    String full_str = full ? "FIFO-full" : "FIFO-not-full";

    bool empty = (ic_status_contents >> 6) & 0x1;
    String empty_str = empty ? "FIFO-empty" : "FIFO-not-empty";

    uint8_t fifo_space = ic_status_contents & 0x3F;
    String fifo_space_str = String(fifo_space) + "-space";

    if (verbose) {
        Serial.println("IC_STATUS (0x" + String(ic_status, HEX) + "):");
        Serial.println("  state: " + state_str);
        Serial.println("  ovv: " + ovv_str);
        Serial.println("  ovt: " + ovt_str);
        Serial.println("  full: " + full_str);
        Serial.println("  empty: " + empty_str);
        Serial.println("  fifo_space: " + fifo_space_str);
    } else {
        Serial.print("IC_STATUS (0x" + String(ic_status, HEX) + "): ");
        Serial.println(state_str + "  " + ovv_str + "  " + ovt_str + "  " + full_str + "  " + empty_str + "  " + fifo_space_str);
    }
}

bool BOS1901::hasFifoSpace() {
    auto ic_status = getRegisterContents(REG_IC_STATUS);
    auto ic_status_contents = getContentOfResponse(ic_status);

    bool full = (ic_status_contents >> 7) & 0x1;
    return !full;
}

bool BOS1901::isFifoEmpty() {
    auto ic_status = getRegisterContents(REG_IC_STATUS);
    auto ic_status_contents = getContentOfResponse(ic_status);

    bool empty = (ic_status_contents >> 6) & 0x1; 
    return empty;
}

uint16_t BOS1901::getRemainingFifoSpace() {
    auto ic_status = getRegisterContents(REG_IC_STATUS);
    auto ic_status_contents = getContentOfResponse(ic_status);

    bool empty = (ic_status_contents >> 6) & 0x1;
    uint8_t fifo_space = ic_status_contents & 0x3F;
    return (empty && fifo_space == 0) ? 64 : fifo_space;
}

void BOS1901::print_reg_sense(bool verbose) {
    auto sense = getRegisterContents(REG_SENSE);
    auto sense_contents = getContentOfResponse(sense);

    uint8_t state = (sense_contents >> 10) & 0x3;
    String state_str = "Unknown";
    if (state == 0x0) state_str = "IDLE";
    else if (state == 0x1) state_str = "CALIBRATION";
    else if (state == 0x2) state_str = "RUN";
    else if (state == 0x3) state_str = "ERROR";

    uint16_t v_feedback = sense_contents & 0x3FF;
    double v_feedback_v = computeSensedVoltage(v_feedback);
    String v_feedback_str = String(v_feedback) + "-" + String(v_feedback_v, 2) + "V";

    if (verbose) {
        Serial.println("SENSE (0x" + String(sense, HEX) + "):");
        Serial.println("  state: " + state_str);
        Serial.println("  v_feedback: " + v_feedback_str);
    } else {
        Serial.print("SENSE (0x" + String(sense, HEX) + "): ");
        Serial.println(state_str + "  " + v_feedback_str);
    }
}

void BOS1901::print_reg_trim(bool verbose) {
    auto trim = getRegisterContents(REG_TRIM);
    auto trim_contents = getContentOfResponse(trim);

    uint8_t trim_rw = (trim_contents >> 10) & 0x3;
    String trim_rw_str = "Unknown";
    if (trim_rw == 0x0) trim_rw_str = "(0x0) Trim-default";
    else if (trim_rw == 0x1) trim_rw_str = "(0x1) Latch HW Fusing to Trim Block";
    else if (trim_rw == 0x2) trim_rw_str = "(0x2) Read value from trim block";
    else if (trim_rw == 0x3) trim_rw_str = "(0x3) Write software trim value";
    String trim_rw_str_concise = "0x" + String(trim_rw, HEX);

    bool sdobp = (trim_contents >> 9) & 0x1;
    String sdobp_str = sdobp ? "Internal clock to SDO pin" : "Keep SDO clock-default";

    uint8_t trim_osc = (trim_contents >> 3) & 0x2f;
    int16_t trim_osc_int = UpscaleTwosComplement((int16_t) trim_osc, 6);
    String trim_osc_str = String(trim_osc_int);

    uint8_t trim_reg = trim_contents & 0x7;
    int16_t trim_reg_int = UpscaleTwosComplement((int16_t) trim_reg, 3);
    double trim_reg_v = 1.8 + (0.022) * trim_reg_int;
    String trim_reg_str = String(trim_reg_int) + "-" + String(trim_reg_v, 2) + "V";

    if (verbose) {
        Serial.println("TRIM (0x" + String(trim, HEX) + "):");
        Serial.println("  trim_rw: " + trim_rw_str);
        Serial.println("  sdobp: " + sdobp_str);
        Serial.println("  trim_osc: " + trim_osc_str);
        Serial.println("  trim_reg: " + trim_reg_str);
    } else {
        Serial.print("TRIM (0x" + String(trim, HEX) + "): ");
        Serial.println(trim_rw_str_concise + "  " + sdobp_str + "  " + trim_osc_str + "  " + trim_reg_str);
    }
}

void BOS1901::scanRegisters(bool verbose) {
    print_reg_reference(verbose);
    print_reg_ion_bl(verbose);
    print_reg_deadtime(verbose);
    print_reg_kp(verbose);
    print_reg_kpa_ki(verbose);
    print_reg_config(verbose);
    print_reg_parcap(verbose);
    print_reg_sup_rise(verbose);
    print_reg_dac(verbose);
    print_reg_ic_status(verbose);
    print_reg_sense(verbose);
    print_reg_trim(verbose);
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

float BOS1901::senseVoltage() {

    // Set chip to SENSE, which mutes the waveform output, allowing 
    // the voltage to move freely and for us to get a reading.
    //   >> This also sets the _outputRegister to REG_SENSE and 
    //      _outputEnabled to true
    setSENSE(true);

    // Read the output
    uint16_t data = getRegisterContents(
        REG_SENSE, // We want to read SENSE register
        false,    // don't allow for cached data
        REG_SENSE // anticipate continued stream of sensing
    );
    uint16_t vfeedback = data & 0x3ff;
    auto vnet = vfeedback - _feedbackAdcOffset;
    return computeSensedVoltage(vnet);
}
