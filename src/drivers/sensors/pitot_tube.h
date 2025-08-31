#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

namespace drivers {

// Sensor data structure
struct pitot_data {
    float pressure_psi;         // Differential pressure in PSI
    float temperature_c;        // Temperature in Celsius
    float airspeed_ms;          // Airspeed in m/s
    float airspeed_mph;         // Airspeed in mph
    bool valid;
};

class PitotTube {
private:

    
    I2CBus* i2c_bus;
    bool initialized;
    pitot_data _data;
    bool _data_ready;
    
    // Calibration
    float zero_offset_psi;
    bool calibrated;
    
    // Pressure range (typically 1.0 PSI for differential sensors)
    float pressure_range;
    
    float calculate_airspeed_ms(float diff_pressure_psi, float air_density);

public:
    PitotTube() : i2c_bus(nullptr), initialized(false), _data_ready(false), 
                 zero_offset_psi(0.0f), calibrated(false), pressure_range(1.0f) {
        _data.valid = false;
    }
    
    bool init(I2CBus* bus, float range_psi = 1.0f);
    bool calibrate_zero(int num_samples = 50);  // Calibrate when stationary
    bool update();                               // Reads from sensor
    pitot_data get_data();                   // Returns cached data
    void clear();                                // Clears data ready flag
};

} // namespace drivers