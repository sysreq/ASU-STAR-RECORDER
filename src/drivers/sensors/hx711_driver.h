#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

namespace drivers {
#define REG_DATA_MSB    0x00
#define REG_DATA_MID    0x01
#define REG_DATA_LSB    0x02
#define REG_CONTROL     0x03
#define REG_TARE        0x04

struct hx711_data {
    int32_t raw_value;  // Raw ADC reading
    float force;        // Converted force in Newtons
    bool valid;
};

class HX711 {
private:
    I2CBus* i2c_bus;
    bool initialized;
    hx711_data _data;
    bool _data_ready;
    int32_t tare_offset;
    float calibration_factor;  // ADC units to Newtons

public:
    HX711() : i2c_bus(nullptr), initialized(false), _data_ready(false), 
              tare_offset(0), calibration_factor(1.0f) {
        _data.valid = false;
    }
    
    bool init(I2CBus* bus);
    bool update();              // Reads from sensor, returns true if new data
    hx711_data get_data();      // Returns cached data
    void clear();               // Clears data ready flag
    void tare();                // Zero the sensor
    void set_calibration(float factor) { calibration_factor = factor; }
};

} // namespace drivers