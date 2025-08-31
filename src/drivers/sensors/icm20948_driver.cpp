#include "icm20948_driver.h"
#include "config/config.h"

namespace drivers {

bool ICM20948::init(I2CBus* bus) {
    using config::i2c::addresses::ICM20948_ADDR;
    using config::icm20948::ACCEL_RANGE;
    using config::icm20948::GYRO_RANGE;
    
    this->i2c_bus = bus;
    
    // Check if device is present
    if (!i2c_bus->device_present(ICM20948_ADDR)) {
        printf("ICM20948: Device not found at address 0x%02X\n", ICM20948_ADDR);
        return false;
    }
    
    // Select bank 0
    if (!select_bank(0)) {
        printf("ICM20948: Failed to select bank 0\n");
        return false;
    }
    
    // Check chip ID (should be 0xEA for ICM-20948)
    uint8_t chip_id;
    if (!i2c_bus->read_register(ICM20948_ADDR, REG_WHO_AM_I, &chip_id, 1)) {
        printf("ICM20948: Failed to read chip ID\n");
        return false;
    }
    
    if (chip_id != 0xEA) {
        printf("ICM20948: Wrong chip ID: 0x%02X (expected 0xEA)\n", chip_id);
        return false;
    }
    
    // Reset device
    if (!i2c_bus->write_register(ICM20948_ADDR, REG_PWR_MGMT_1, 0x80)) {
        printf("ICM20948: Failed to reset device\n");
        return false;
    }
    sleep_ms(100);
    
    // Wake up device, auto select clock
    if (!i2c_bus->write_register(ICM20948_ADDR, REG_PWR_MGMT_1, 0x01)) {
        printf("ICM20948: Failed to wake device\n");
        return false;
    }
    sleep_ms(20);
    
    // Enable all sensors
    if (!i2c_bus->write_register(ICM20948_ADDR, REG_PWR_MGMT_2, 0x00)) {
        printf("ICM20948: Failed to enable sensors\n");
        return false;
    }
    
    // OPTIMIZATION: Batch bank 2 operations
    // Select bank 2 for both accel and gyro config
    if (!select_bank(2)) {
        printf("ICM20948: Failed to select bank 2\n");
        return false;
    }
    
    // Configure accelerometer with compile-time range
    if (!i2c_bus->write_register(ICM20948_ADDR, REG_ACCEL_CONFIG, ACCEL_RANGE << 1)) {
        printf("ICM20948: Failed to configure accelerometer\n");
        return false;
    }
    
    // Configure gyroscope with compile-time range (still in bank 2)
    if (!i2c_bus->write_register(ICM20948_ADDR, REG_GYRO_CONFIG_1, GYRO_RANGE << 1)) {
        printf("ICM20948: Failed to configure gyroscope\n");
        return false;
    }
    
    // Return to bank 0 for data reading
    if (!select_bank(0)) {
        printf("ICM20948: Failed to return to bank 0\n");
        return false;
    }
    
    initialized = true;
    _data.valid = false;
    _data_ready = false;
    
    printf("ICM20948: Initialized successfully\n");
    return true;
}

bool ICM20948::update() {
    using config::i2c::addresses::ICM20948_ADDR;
    using config::icm20948::ACCEL_SCALE;
    using config::icm20948::GYRO_SCALE;
    
    if (!initialized) {
        _data_ready = false;
        return false;
    }
    
    // OPTIMIZATION: Only select bank 0 if not already there
    if (current_bank != 0) {
        if (!select_bank(0)) {
            _data_ready = false;
            return false;
        }
    }
    
    // OPTIMIZATION: Read only accel + gyro data (12 bytes instead of 20)
    uint8_t raw_data[12];
    if (!i2c_bus->read_register(ICM20948_ADDR, REG_ACCEL_XOUT_H, raw_data, 12)) {
        _data_ready = false;
        return false;
    }
    
    // Parse accelerometer data (bytes 0-5)
    int16_t accel_x_raw = utils::merge_bytes<int16_t>(raw_data[0], raw_data[1]);
    int16_t accel_y_raw = utils::merge_bytes<int16_t>(raw_data[2], raw_data[3]);
    int16_t accel_z_raw = utils::merge_bytes<int16_t>(raw_data[4], raw_data[5]);
    
    // Parse gyroscope data (bytes 6-11)
    int16_t gyro_x_raw = utils::merge_bytes<int16_t>(raw_data[6], raw_data[7]);
    int16_t gyro_y_raw = utils::merge_bytes<int16_t>(raw_data[8], raw_data[9]);
    int16_t gyro_z_raw = utils::merge_bytes<int16_t>(raw_data[10], raw_data[11]);
    
    // Convert to SI units using compile-time scale factors
    _data.accel_x = accel_x_raw * ACCEL_SCALE;
    _data.accel_y = accel_y_raw * ACCEL_SCALE;
    _data.accel_z = accel_z_raw * ACCEL_SCALE;
    
    _data.gyro_x = gyro_x_raw * GYRO_SCALE;
    _data.gyro_y = gyro_y_raw * GYRO_SCALE;
    _data.gyro_z = gyro_z_raw * GYRO_SCALE;
    
    _data.valid = true;
    _data_ready = true;
    
    return true;
}

icm20948_data ICM20948::get_data() {
    return _data;
}

void ICM20948::clear() {
    _data_ready = false;
}

bool ICM20948::select_bank(uint8_t bank) {
    using config::i2c::addresses::ICM20948_ADDR;
    
    // OPTIMIZATION: Only switch if different bank
    if (current_bank == bank) {
        return true;
    }
    
    if (i2c_bus->write_register(ICM20948_ADDR, REG_BANK_SEL, (bank & 0x03) << 4)) {
        current_bank = bank;
        return true;
    }
    return false;
}

} // namespace drivers