#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

namespace drivers {

// Register addresses
#define BMP581_REG_CHIP_ID      0x01
#define BMP581_REG_CHIP_STATUS  0x11
#define BMP581_REG_TEMP_DATA    0x1D  // 3 bytes
#define BMP581_REG_PRESS_DATA   0x20  // 3 bytes
#define BMP581_REG_INT_STATUS   0x27
#define BMP581_REG_STATUS       0x28  // Data ready status
#define BMP581_REG_PWR_CTRL     0x33
#define BMP581_REG_OSR_CONFIG   0x36
#define BMP581_REG_ODR_CONFIG   0x37
#define BMP581_REG_OSR_EFF      0x38
#define BMP581_REG_CMD          0x7E  // Command register for soft reset

// Sensor data structure
struct bmp581_data {
    float temperature;  // Celsius
    float pressure;     // Pascals
    float altitude;     // Meters
    bool valid;
};

class BMP581 {
private:
    I2CBus* i2c_bus;
    bool initialized;
    bmp581_data _data;
    bool _data_ready;
    
    float calculate_altitude(float pressure);

public:
    BMP581() : i2c_bus(nullptr), initialized(false), _data_ready(false) {
        _data.valid = false;
    }
    
    bool init(I2CBus* bus);
    bool update();              // Reads from sensor, returns true if new data
    bmp581_data get_data();     // Returns cached data
    void clear();               // Clears data ready flag
};

} // namespace drivers