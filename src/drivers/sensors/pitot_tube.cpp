#include "pitot_tube.h"
#include "config/config.h"

namespace drivers {

bool PitotTube::init(I2CBus* bus, float range_psi) {
    using config::i2c::addresses::PITOT;
    this->i2c_bus = bus;
    this->pressure_range = range_psi;
    
    // Check if device is present
    if (!i2c_bus->device_present(PITOT)) {
        printf("MS4525DO: Device not found at address 0x%02X\n", PITOT);
        return false;
    }
    
    // Read initial data to verify communication
    uint8_t buffer[4];
    if (i2c_bus->read_blocking(PITOT, buffer, 4) != 4) {
        printf("MS4525DO: Failed to read initial data\n");
        return false;
    }
    
    // Check status bits (top 2 bits of first byte)
    uint8_t status = (buffer[0] & 0xC0) >> 6;
    if (status == 3) {  // Diagnostic condition
        printf("MS4525DO: Sensor in diagnostic condition\n");
        return false;
    }
    
    initialized = true;
    _data.valid = false;
    _data_ready = false;
    calibrated = false;
    
    printf("MS4525DO: Initialized successfully (range: %.1f PSI)\n", pressure_range);
    return true;
}

bool PitotTube::calibrate_zero(int num_samples) {
    if (!initialized) {
        return false;
    }
    
    float pressure_sum = 0.0f;
    int valid_readings = 0;
    
    for (int i = 0; i < num_samples; i++) {
        uint8_t buffer[4];
        
        if (i2c_bus->read_blocking(config::i2c::addresses::PITOT, buffer, 4) == 4) {
            uint8_t status = (buffer[0] & 0xC0) >> 6;
            
            // Only use normal readings (status = 0)
            if (status == 0) {
                uint16_t pressure_raw = ((buffer[0] & 0x3F) << 8) | buffer[1];
                float pressure_normalized = (float)(pressure_raw - 1638) / (14745.0f - 1638.0f);
                pressure_sum += pressure_normalized * pressure_range;
                valid_readings++;
            }
        }
        
        sleep_ms(20);
    }
    
    if (valid_readings < (num_samples / 2)) {
        printf("MS4525DO: Calibration failed - insufficient valid readings\n");
        return false;
    }
    
    zero_offset_psi = pressure_sum / valid_readings;
    calibrated = true;
    
    printf("MS4525DO: Calibration complete (offset: %.6f PSI)\n", zero_offset_psi);
    return true;
}

bool PitotTube::update() {
    using config::i2c::addresses::PITOT;
    using config::pitot_tube::STANDARD_AIR_DENSITY;
    using config::pitot_tube::MS_TO_MPH;

    if (!initialized) {
        _data_ready = false;
        return false;
    }
    
    uint8_t buffer[4];
    
    // Read 4 bytes from sensor
    if (i2c_bus->read_blocking(PITOT, buffer, 4) != 4) {
        _data_ready = false;
        return false;
    }
    
    // Parse status (top 2 bits)
    uint8_t status = (buffer[0] & 0xC0) >> 6;
    
    // Status: 0=normal, 1=command mode, 2=stale data, 3=diagnostic
    if (status == 2 || status == 3) {
        _data_ready = false;
        return false;
    }
    
    // Extract 14-bit pressure data
    uint16_t pressure_raw = ((buffer[0] & 0x3F) << 8) | buffer[1];
    
    // Extract 11-bit temperature data
    uint16_t temp_raw = (buffer[2] << 3) | ((buffer[3] & 0xE0) >> 5);
    
    // Convert pressure to PSI (10% to 90% output range)
    float pressure_normalized = (float)(pressure_raw - 1638) / (14745.0f - 1638.0f);
    _data.pressure_psi = pressure_normalized * pressure_range;
    
    // Apply calibration if available
    if (calibrated) {
        _data.pressure_psi -= zero_offset_psi;
    }
    
    // Convert temperature to Celsius
    _data.temperature_c = (temp_raw * 200.0f / 2047.0f) - 50.0f;
    
    // Calculate airspeed
    _data.airspeed_ms = calculate_airspeed_ms(_data.pressure_psi, STANDARD_AIR_DENSITY);
    _data.airspeed_mph = _data.airspeed_ms * MS_TO_MPH;
    
    _data.valid = true;
    _data_ready = true;
    
    return true;
}

pitot_data PitotTube::get_data() {
    return _data;
}

void PitotTube::clear() {
    _data_ready = false;
}

float PitotTube::calculate_airspeed_ms(float diff_pressure_psi, float air_density) {
    using config::pitot_tube::PSI_TO_PA;
    // Convert PSI to Pascals
    float diff_pressure_pa = diff_pressure_psi * PSI_TO_PA;
    
    // Handle negative or very small pressures
    if (diff_pressure_pa <= 0.0f) {
        return 0.0f;
    }
    
    // Bernoulli's equation: V = √(2 * ΔP / ρ)
    return sqrtf((2.0f * diff_pressure_pa) / air_density);
}

} // namespace drivers