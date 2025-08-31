#pragma once

// Project Omni-Header
#include "config/all_headers.h"

namespace drivers {

class I2CBus {
public:
    static constexpr size_t MAX_I2C_TRANSFER = 64;
    
    I2CBus() : _i2c(nullptr), _initialized(false) {}
    
    bool init(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint baudrate = 400000) {
        if (_initialized) return true;
        
        _i2c = i2c_port;
        
        // Initialize I2C
        i2c_init(_i2c, baudrate);
        
        // Setup pins
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
        
        _initialized = true;
        return true;
    }
    
    // Check if device is present at address
    bool device_present(uint8_t addr, uint32_t timeout_us = 100000) {
        uint8_t dummy;
        return read_timeout(addr, &dummy, 1, timeout_us) > 0;
    }
    
    // Read register(s) from device
    bool read_register(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
        if (write_blocking(addr, &reg, 1, true) < 1) {
            return false;
        }

        auto res = read_blocking(addr, data, len) == (int)len;
        return res;
    }
    
    // Write single register
    bool write_register(uint8_t addr, uint8_t reg, uint8_t value) {
        uint8_t buf[2] = {reg, value};
        return write_blocking(addr, buf, 2) == 2;
    }
    
    // Write multiple bytes to register
    bool write_register(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
        if (len + 1 > MAX_I2C_TRANSFER) {
            return false;  // Transfer too large
        }
        
        uint8_t buf[MAX_I2C_TRANSFER];
        buf[0] = reg;
        memcpy(buf + 1, data, len);
        return write_blocking(addr, buf, len + 1) == (int)(len + 1);
    }
    
    // Raw I2C operations with timeout
    int read_timeout(uint8_t addr, uint8_t* data, size_t len, uint32_t timeout_us = 100000) {
        return i2c_read_timeout_us(_i2c, addr, data, len, false, timeout_us);
    }
    
    int write_timeout(uint8_t addr, const uint8_t* data, size_t len, uint32_t timeout_us = 100000) {
        return i2c_write_timeout_us(_i2c, addr, data, len, false, timeout_us);
    }
    
    // Raw I2C blocking operations
    int read_blocking(uint8_t addr, uint8_t* data, size_t len, bool nostop = false) {
        return i2c_read_blocking(_i2c, addr, data, len, nostop);
    }
    
    int write_blocking(uint8_t addr, const uint8_t* data, size_t len, bool nostop = false) {
        return i2c_write_blocking(_i2c, addr, data, len, nostop);
    }
    
    i2c_inst_t* get() { return _i2c; }
    bool is_initialized() { return _initialized; }
    
private:
    i2c_inst_t* _i2c;
    bool _initialized;
};

} // namespace drivers