#pragma once

// Project Omni-Header
#include "config/all_headers.h"

// Project
#include "i2c_bus.h"

// sh2 Lib Files
extern "C" {
    #include "sh2.h"
    #include "sh2_SensorValue.h"
    #include "sh2_err.h"
}

namespace drivers {

struct bno085_data {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float accel_x;
    float accel_y;
    float accel_z;
    bool valid;
};

class BNO085 {
public:
    BNO085() = default;
    
    bool init(I2CBus* bus);
    bool update();
    bno085_data get_data();
    void clear();

private:
    I2CBus* _bus;
    sh2_Hal_t _hal;
    sh2_SensorValue_t _sensor_value;
    bno085_data _data;
    bool _data_ready;
    
    static int hal_open(sh2_Hal_t* self);
    static void hal_close(sh2_Hal_t* self);
    static int hal_read(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t_us);
    static int hal_write(sh2_Hal_t* self, uint8_t* buf, unsigned len);
    static uint32_t hal_get_time_us(sh2_Hal_t* self);
    
    static void hal_callback(void* cookie, sh2_AsyncEvent_t* event);
    static void sensor_handler(void* cookie, sh2_SensorEvent_t* event);
    
    bool enable_sensor(sh2_SensorId_t id, uint32_t interval_us);
};

} // namespace drivers