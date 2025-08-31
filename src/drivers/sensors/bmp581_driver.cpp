#include "bmp581_driver.h"
#include "config/config.h"

namespace drivers {

bool BMP581::init(I2CBus* bus) {
    using config::i2c::addresses::BMP581_ADDR;
    this->i2c_bus = bus;
    
    // Check if device is present
    if (!i2c_bus->device_present(BMP581_ADDR)) {
        printf("BMP581: Device not found at address 0x%02X\n", BMP581_ADDR);
        return false;
    }
    
    // Check chip ID (should be 0x50 for BMP581)
    uint8_t chip_id;
    if (!i2c_bus->read_register(BMP581_ADDR, BMP581_REG_CHIP_ID, &chip_id, 1)) {
        printf("BMP581: Failed to read chip ID\n");
        return false;
    }
    
    if (chip_id != 0x50) {
        printf("BMP581: Wrong chip ID: 0x%02X (expected 0x50)\n", chip_id);
        return false;
    }
    
    // Perform soft reset (0xB6 is the standard Bosch reset command)
    if (!i2c_bus->write_register(BMP581_ADDR, BMP581_REG_CMD, 0xB6)) {
        printf("BMP581: Failed to reset\n");
        return false;
    }
    sleep_ms(10);  // Wait for reset to complete
    
    // OSR register: x2 oversampling for temperature and pressure, enable pressure
    // Bit 6 = pressure enable, Bits [5:3] = temp OSR, [2:0] = press OSR, 001 = x2
    if (!i2c_bus->write_register(BMP581_ADDR, BMP581_REG_OSR_CONFIG, 0x49)) {  // 0x40 | 0x09
        printf("BMP581: Failed to configure oversampling\n");
        return false;
    }
    
    // ODR register: 50Hz output data rate (normal/continuous mode)
    // 0xBD = 0x80 | 0x3C | 0x01 (Deep disable + 50Hz + Normal mode)
    if (!i2c_bus->write_register(BMP581_ADDR, BMP581_REG_ODR_CONFIG, 0xBD)) {
        printf("BMP581: Failed to configure output data rate\n");
        return false;
    }
    
    sleep_ms(50);  // Wait for sensor to stabilize and first measurement
    
    initialized = true;
    _data.valid = false;
    _data_ready = false;
    
    printf("BMP581: Initialized successfully\n");
    return true;
}

bool BMP581::update() {
    using config::i2c::addresses::BMP581_ADDR;

    if (!initialized) {
        _data_ready = false;
        return false;
    }
    
    // In normal mode, the sensor continuously updates at the configured ODR
    // No need to check data ready status - just read the latest values
    
    // Read temperature data (3 bytes)
    uint8_t temp_data[3];
    if (!i2c_bus->read_register(BMP581_ADDR, BMP581_REG_TEMP_DATA, temp_data, 3)) {
        _data_ready = false;
        return false;
    }
    
    // Read pressure data (3 bytes)
    uint8_t press_data[3];
    if (!i2c_bus->read_register(BMP581_ADDR, BMP581_REG_PRESS_DATA, press_data, 3)) {
        _data_ready = false;
        return false;
    }
    
    // Convert temperature (24-bit signed, LSB first in registers)
    int32_t raw_temp = utils::merge_bytes<int32_t>(temp_data[2], temp_data[1], temp_data[0]);
    _data.temperature = raw_temp / 65536.0f;
    
    // Convert pressure (24-bit unsigned, LSB first in registers)
    uint32_t raw_press = (uint32_t)press_data[0] | ((uint32_t)press_data[1] << 8) | ((uint32_t)press_data[2] << 16);
    _data.pressure = raw_press / 64.0f;
    
    // Calculate altitude using standard atmosphere model
    _data.altitude = calculate_altitude(_data.pressure);
    
    _data.valid = true;
    _data_ready = true;
    
    return true;
}

bmp581_data BMP581::get_data() {
    return _data;
}

void BMP581::clear() {
    _data_ready = false;
}

float BMP581::calculate_altitude(float pressure) {
    const float sea_level_pressure = 101325.0f;  // Pa
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

} // namespace drivers