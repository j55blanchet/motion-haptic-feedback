

#ifndef BOS1901_H
#define BOS1901_H

#include <SPI.h>

class BOS1901 {

    static const uint8_t REG_REFERENCE = 0x00; // R/W. Input of the FIFO. Desired amplitude of output in 12-bit two's complement format.
    static const uint8_t REG_ION_BL    = 0x01; // R/W. Sets boost converter maximum switching frequency, blanking time, and min current to turn on HS
    static const uint8_t REG_DEADTIME  = 0x02; // R/W. Sets power switch deadtime for low-side and high-side switch.
    static const uint8_t REG_KP        = 0x03; // R/W. Sets square wave playback and proportional gain parameter.
    static const uint8_t REG_KPA_KI    = 0x04; // R/W. Sets internal parameters KIBase and KPA.
    static const uint8_t REG_CONFIG    = 0x05; // R/W. Used to configure output register, reset chip, enable / disable waveform plaback, set waveform playback speed..
    static const uint8_t REG_PARCAP    = 0x06; // R/W. Used to set power configuration params (see datasheet)
    static const uint8_t REG_SUP_RISE  = 0x07; // R/W. Used to config chip into sensing mode.
    static const uint8_t REG_DAC       = 0x08; // Ignored - only has internal calibration params
    static const uint8_t REG_IC_STATUS = 0x0C; // Read only, used for getting IC status
    static const uint8_t REG_SENSE     = 0x0D; // Read only. Gets controller state and sensed voltage.
    static const uint8_t REG_TRIM      = 0x0E; // R/W. Used for TRIM control?

    static const uint8_t REG_LOCK_DISABLE = 0x0;
    static const uint8_t REG_LOCK_ENABLE = 0x1;

    SPIClass &_spi;
    uint8_t _chipSelectPin;
    uint8_t _playbackSpeed;
    uint8_t _vddSupRise;
    bool _lockRegisters;
    bool _outputEnabled;

    uint16_t getRegisterContents(uint8_t reg_addr);
    uint16_t getContentOfResponse(uint16_t data);
    uint8_t getRegisterOfResponse(uint16_t data);

    static uint16_t makeCommand(uint8_t reg_addr, uint16_t data);
    uint16_t transfer(uint16_t write_command);
    uint16_t setConfig(uint8_t output_reg_select, bool enable_waveform_playback, bool override_unlock_registers=false);
    void setSENSE(bool enableSensing);
    uint16_t writeWaveform(uint16_t waveForm);

    public: 
        // Playback speeds in ksps (for use in the config secion)
        static const uint8_t PLAYBACK_SPEED_1024_ksps = 0x0;
        static const uint8_t PLAYBACK_SPEED_512_ksps  = 0x1;
        static const uint8_t PLAYBACK_SPEED_256_ksps  = 0x2;
        static const uint8_t PLAYBACK_SPEED_128_ksps  = 0x3;
        static const uint8_t PLAYBACK_SPEED_64_ksps   = 0x4;
        static const uint8_t PLAYBACK_SPEED_32_ksps   = 0x5;
        static const uint8_t PLAYBACK_SPEED_16_ksps   = 0x6;
        static const uint8_t PLAYBACK_SPEED_8_ksps    = 0x7;

        BOS1901(
            SPIClass &spi, 
            uint8_t chipSelectPin, 
            uint8_t playback_spd, 
            bool lock_registers,
            float supply_voltage);

        void reset();
        uint16_t getADCoffset();
        uint16_t senseVoltage();
        void scanRegisters(bool verbose);

        void print_reg_reference(bool verbose);
        void print_reg_ion_bl(bool verbose);
        void print_reg_deadtime(bool verbose);
        void print_reg_kp(bool verbose);
        void print_reg_kpa_ki(bool verbose);
        void print_reg_config(bool verbose);
        void print_reg_parcap(bool verbose);
        void print_reg_sup_rise(bool verbose);
        void print_reg_dac(bool verbose);
        void print_reg_ic_status(bool verbose);
        void print_reg_sense(bool verbose);
        void print_reg_trim(bool verbose);
};

#endif