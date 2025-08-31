#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

namespace drivers {

// Register addresses
#define REG_WHO_AM_I        0x00
#define REG_USER_CTRL       0x03
#define REG_PWR_MGMT_1      0x06
#define REG_PWR_MGMT_2      0x07
#define REG_GYRO_CONFIG_1   0x01
#define REG_ACCEL_CONFIG    0x14
#define REG_ACCEL_CONFIG_2  0x15
#define REG_ACCEL_XOUT_H    0x2D
#define REG_GYRO_XOUT_H     0x33
#define REG_BANK_SEL        0x7F

// Sensor data structure (temperature removed)
struct icm20948_data {
    float accel_x;  // m/s^2
    float accel_y;  // m/s^2
    float accel_z;  // m/s^2
    float gyro_x;   // rad/s
    float gyro_y;   // rad/s
    float gyro_z;   // rad/s
    bool valid;
};

class ICM20948 {
private:
    I2CBus* i2c_bus;
    bool initialized;
    icm20948_data _data;
    bool _data_ready;
    uint8_t current_bank;  // Cache current bank to avoid redundant switches
    
    bool select_bank(uint8_t bank);

public:
    ICM20948() : i2c_bus(nullptr), initialized(false), _data_ready(false), current_bank(0xFF) {
        _data.valid = false;
    }
    
    bool init(I2CBus* bus);
    bool update();              // Reads from sensor, returns true if new data
    icm20948_data get_data();  // Returns cached data
    void clear();               // Clears data ready flag
};

} // namespace drivers