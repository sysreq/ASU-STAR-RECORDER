#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

namespace drivers {

// Register addresses
#define REG_CHIP_ID     0x00
#define REG_DATA        0x04
#define REG_PWR_CTRL    0x1B
#define REG_OSR         0x1C
#define REG_ODR         0x1D
#define REG_CALIB_DATA  0x31

// Calibration data structure
struct bmp390_calib {
    uint16_t T1;
    uint16_t T2;
    int8_t   T3;
    int16_t  P1;
    int16_t  P2;
    int8_t   P3;
    int8_t   P4;
    uint16_t P5;
    uint16_t P6;
    int8_t   P7;
    int8_t   P8;
    int16_t  P9;
    int8_t   P10;
    int8_t   P11;
};

// Sensor data structure
struct bmp390_data {
    float temperature;  // Celsius
    float pressure;     // Pascals
    float altitude;     // Meters
    bool valid;        // Added for consistency with BNO085
};

class BMP390 {
private:
    I2CBus* i2c_bus;
    struct bmp390_calib calib;
    int64_t t_fine;  // Fine temperature value for pressure calculation
    bool initialized;
    bmp390_data _data;     // Cached sensor data
    bool _data_ready;      // Flag for new data availability

    bool read_calibration();
    int64_t compensate_temperature(uint32_t raw_temp);
    uint64_t compensate_pressure(uint32_t raw_press);
    float calculate_altitude(float pressure);

public:
    BMP390() : i2c_bus(nullptr), t_fine(0), initialized(false), _data_ready(false) {
        _data.valid = false;
    }
    
    bool init(I2CBus* bus);
    bool update();              // Reads from sensor, returns true if new data
    bmp390_data get_data();     // Returns cached data
    void clear();               // Clears data ready flag
};

} // namespace drivers