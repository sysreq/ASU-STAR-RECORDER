#include "bmp390_driver.h"
#include "config/config.h"

namespace drivers {

bool BMP390::init(I2CBus* bus) {
    using config::i2c::addresses::BMP390_ADDR;
    this->i2c_bus = bus;
    
    // Check if device is present
    if (!i2c_bus->device_present(BMP390_ADDR)) {
        printf("BMP390: Device not found at address 0x%02X\n", BMP390_ADDR);
        return false;
    }
    
    // Check chip ID (should be 0x60 for BMP390)
    uint8_t chip_id;
    if (!i2c_bus->read_register(BMP390_ADDR, REG_CHIP_ID, &chip_id, 1)) {
        printf("BMP390: Failed to read chip ID\n");
        return false;
    }
    
    if (chip_id != 0x60) {
        printf("BMP390: Wrong chip ID: 0x%02X (expected 0x60)\n", chip_id);
        return false;
    }
    
    // Read calibration data
    if (!read_calibration()) {
        printf("BMP390: Failed to read calibration data\n");
        return false;
    }
    
    // OSR register: x2 oversampling for temperature and pressure
    if (!i2c_bus->write_register(BMP390_ADDR, REG_OSR, 0x09)) {  // temp_os=x2(001), press_os=x2(001)
        printf("BMP390: Failed to configure oversampling\n");
        return false;
    }
    
    // ODR register: 50Hz output data rate
    if (!i2c_bus->write_register(BMP390_ADDR, REG_ODR, 0x02)) {
        printf("BMP390: Failed to configure output data rate\n");
        return false;
    }
    
    // Power control: enable temperature and pressure, normal mode
    if (!i2c_bus->write_register(BMP390_ADDR, REG_PWR_CTRL, 0x33)) {
        printf("BMP390: Failed to configure power control\n");
        return false;
    }
    
    sleep_ms(10);  // Wait for sensor to stabilize
    
    initialized = true;
    _data.valid = false;
    _data_ready = false;
    
    printf("BMP390: Initialized successfully\n");
    return true;
}

bool BMP390::update() {
    using config::i2c::addresses::BMP390_ADDR;

    if (!initialized) {
        _data_ready = false;
        return false;
    }
    
    uint8_t raw_data[6];
    if (!i2c_bus->read_register(BMP390_ADDR, REG_DATA, raw_data, 6)) {
        _data_ready = false;
        return false;
    }

    // Convert to 24-bit unsigned values
    uint32_t raw_press =  utils::merge_bytes<uint32_t>(raw_data[2], raw_data[1], raw_data[0]);
    uint32_t raw_temp =  utils::merge_bytes<uint32_t>(raw_data[5], raw_data[4], raw_data[3]);
    
    // Compensate temperature (must be done first)
    int64_t temp_comp = compensate_temperature(raw_temp);
    _data.temperature = temp_comp / 100.0f;
    
    // Compensate pressure
    uint64_t press_comp = compensate_pressure(raw_press);
    _data.pressure = press_comp / 100.0f;
    
    // Calculate altitude using standard atmosphere model
    _data.altitude = calculate_altitude(_data.pressure);
    
    _data.valid = true;
    _data_ready = true;
    
    return true;
}

bmp390_data BMP390::get_data() {
    return _data;
}

void BMP390::clear() {
    _data_ready = false;
}

bool BMP390::read_calibration() {
    using config::i2c::addresses::BMP390_ADDR;

    uint8_t calib_data[21];
    if (!i2c_bus->read_register(BMP390_ADDR, REG_CALIB_DATA, calib_data, 21)) {
        return false;
    }
    
    // Parse calibration coefficients (little-endian)
    calib.T1 = (uint16_t)(calib_data[1] << 8) | calib_data[0];
    calib.T2 = (uint16_t)(calib_data[3] << 8) | calib_data[2];
    calib.T3 = (int8_t)calib_data[4];
    calib.P1 = (int16_t)((calib_data[6] << 8) | calib_data[5]);
    calib.P2 = (int16_t)((calib_data[8] << 8) | calib_data[7]);
    calib.P3 = (int8_t)calib_data[9];
    calib.P4 = (int8_t)calib_data[10];
    calib.P5 = (uint16_t)(calib_data[12] << 8) | calib_data[11];
    calib.P6 = (uint16_t)(calib_data[14] << 8) | calib_data[13];
    calib.P7 = (int8_t)calib_data[15];
    calib.P8 = (int8_t)calib_data[16];
    calib.P9 = (int16_t)((calib_data[18] << 8) | calib_data[17]);
    calib.P10 = (int8_t)calib_data[19];
    calib.P11 = (int8_t)calib_data[20];
    
    return true;
}

int64_t BMP390::compensate_temperature(uint32_t raw_temp) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t comp_temp;
    
    // Compensation formula from BMP390 datasheet
    partial_data1 = (int64_t)(raw_temp - ((int64_t)256 * calib.T1));
    partial_data2 = (int64_t)(calib.T2 * partial_data1);
    partial_data3 = (int64_t)(partial_data1 * partial_data1);
    partial_data4 = (int64_t)partial_data3 * calib.T3;
    partial_data5 = (int64_t)((partial_data2 * 262144) + partial_data4);
    partial_data6 = (int64_t)(partial_data5 / 4294967296);
    
    // Store t_fine for pressure calculation
    t_fine = partial_data6;
    
    // Calculate compensated temperature in 0.01 degree Celsius
    comp_temp = (int64_t)((partial_data6 * 25) / 16384);
    
    return comp_temp;
}

uint64_t BMP390::compensate_pressure(uint32_t raw_press) {
    int64_t partial_data1;
    int64_t partial_data2;
    int64_t partial_data3;
    int64_t partial_data4;
    int64_t partial_data5;
    int64_t partial_data6;
    int64_t offset;
    int64_t sensitivity;
    uint64_t comp_press;
    
    // Compensation formula from BMP390 datasheet
    partial_data1 = (int64_t)(t_fine * t_fine);
    partial_data2 = (int64_t)(partial_data1 / 64);
    partial_data3 = (int64_t)((partial_data2 * t_fine) / 256);
    partial_data4 = (int64_t)((calib.P8 * partial_data3) / 32);
    partial_data5 = (int64_t)((calib.P7 * partial_data1) * 16);
    partial_data6 = (int64_t)((calib.P6 * t_fine) * 4194304);
    offset = (int64_t)((calib.P5 * (int64_t)140737488355328) + partial_data4 + partial_data5 + partial_data6);
    
    partial_data2 = (int64_t)((calib.P4 * partial_data3) / 32);
    partial_data4 = (int64_t)((calib.P3 * partial_data1) * 4);
    partial_data5 = (int64_t)((calib.P2 - 16384) * t_fine * 2097152);
    sensitivity = (int64_t)(((calib.P1 - 16384) * (int64_t)70368744177664) + partial_data2 + partial_data4 + partial_data5);
    
    partial_data1 = (int64_t)((sensitivity / 16777216) * raw_press);
    partial_data2 = (int64_t)(calib.P10 * t_fine);
    partial_data3 = (int64_t)(partial_data2 + (65536 * calib.P9));
    partial_data4 = (int64_t)((partial_data3 * raw_press) / 8192);
    partial_data5 = (int64_t)((raw_press * (partial_data4 / 10)) / 512) * 10;
    partial_data6 = (int64_t)(raw_press * raw_press);
    partial_data2 = (int64_t)((calib.P11 * partial_data6) / 65536);
    partial_data3 = (int64_t)((partial_data2 * raw_press) / 128);
    partial_data4 = (int64_t)((offset / 4) + partial_data1 + partial_data5 + partial_data3);
    comp_press = (((uint64_t)partial_data4 * 25) / (uint64_t)1099511627776);
    
    return comp_press;
}

float BMP390::calculate_altitude(float pressure) {
    const float sea_level_pressure = 101325.0f;  // Pa
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

}