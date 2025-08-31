#include "hx711_driver.h"
#include "config/config.h"

namespace drivers {

bool HX711::init(I2CBus* bus) {
    this->i2c_bus = bus;
    
    // Check if device is present
    if (!i2c_bus->device_present(config::i2c::addresses::HX711)) {
        printf("[HX711][XX] Device not found at address 0x%02X\n", config::i2c::addresses::HX711);
        return false;
    }
    
    // Reset the device (typical control register setup)
    // Gain=128 (channel A), normal mode
    if (!i2c_bus->write_register(config::i2c::addresses::HX711, REG_CONTROL, 0x80)) {
        printf("[HX711][XX] Failed to configure device\n");
        return false;
    }
    
    sleep_ms(100);  // Wait for device to stabilize
    
    // Perform initial tare
    tare();
    
    initialized = true;
    _data.valid = false;
    _data_ready = false;
    
    printf("[HX711][OK] Initialized successfully\n");
    return true;
}

bool HX711::update() {
    if (!initialized) {
        _data_ready = false;
        return false;
    }
    
    uint8_t raw_data[3];
    if (!i2c_bus->read_register(config::i2c::addresses::HX711, REG_DATA_MSB, raw_data, 3)) {
        _data_ready = false;
        return false;
    }
    
    // Convert 24-bit signed value
    int32_t raw_uncalib = utils::merge_bytes<int32_t>(raw_data[0], raw_data[1], raw_data[2]);
    _data.raw_value = raw_uncalib - tare_offset;
    _data.force = _data.raw_value * calibration_factor;
    _data.valid = true;
    _data_ready = true;
    
    return true;
}

hx711_data HX711::get_data() {
    return _data;
}

void HX711::clear() {
    _data_ready = false;
}

void HX711::tare() {
    if (!initialized) return;
    
    // Take average of 10 readings for tare
    int32_t sum = 0;
    int valid_readings = 0;
    
    for (int i = 0; i < 10; i++) {
        uint8_t raw_data[3];
        if (i2c_bus->read_register(config::i2c::addresses::HX711, REG_DATA_MSB, raw_data, 3)) {
            int32_t raw = (static_cast<int32_t>(raw_data[0]) << 16) | 
                          (static_cast<int32_t>(raw_data[1]) << 8) | 
                          raw_data[2];
            
            // Sign extend from 24-bit to 32-bit
            if (raw & 0x800000) {
                raw |= 0xFF000000;
            }
            
            sum += raw;
            valid_readings++;
        }
        sleep_ms(10);
    }
    
    if (valid_readings > 0) {
        tare_offset = sum / valid_readings;
    }
}

} // namespace drivers