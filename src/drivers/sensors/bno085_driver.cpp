#include "bno085_driver.h"
#include "config/config.h"

#define DEBUG_I2C_TIMING 0

using config::i2c::addresses::BNO085_ADDR;

namespace drivers {
static BNO085* g_instance = nullptr;

bool BNO085::init(I2CBus* bus) {
    _bus = bus;
    _data_ready = false;
    g_instance = this;
    
    // Check if device is present
    if (!_bus->device_present(BNO085_ADDR)) {
        printf("[BNO085][X] Initialized failed. (Device not found)\n");
        return false;
    }
    
    // Setup HAL
    _hal.open = hal_open;
    _hal.close = hal_close;
    _hal.read = hal_read;
    _hal.write = hal_write;
    _hal.getTimeUs = hal_get_time_us;
    
    // Open SH2 interface
    if (sh2_open(&_hal, hal_callback, this) != SH2_OK) {
        printf("[BNO085][X] Initialized failed. (SH protocol failure)\n");
        return false;
    }
    
    // Register sensor callback
    sh2_setSensorCallback(sensor_handler, this);
    
    // Enable sensors with 10ms update rate
    enable_sensor(SH2_ACCELEROMETER, 50000);
    //enable_sensor(SH2_GYROSCOPE_CALIBRATED, 10000);
    //enable_sensor(SH2_MAGNETIC_FIELD_CALIBRATED, 10000);
    //enable_sensor(SH2_GYRO_INTEGRATED_RV, 50000);
    //enable_sensor(SH2_PRESSURE, 10000);
    
    printf("[BNO085][OK] Initialized successfully.\n");
    return true;
}

bool BNO085::update() {
    _data_ready = false;
    sh2_service();
    return _data_ready;
}

bno085_data BNO085::get_data() {
    return _data;
}

void BNO085::clear() {
    _data_ready = false;
}

bool BNO085::enable_sensor(sh2_SensorId_t id, uint32_t interval_us) {
    sh2_SensorConfig_t config = {};
    config.reportInterval_us = interval_us;
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    
    return sh2_setSensorConfig(id, &config) == SH2_OK;
}

// HAL implementation
int BNO085::hal_open(sh2_Hal_t* self) {
    // Send soft reset
    uint8_t reset_pkt[] = {5, 0, 1, 0, 1};
    for (int i = 0; i < 5; i++) {
        if (g_instance->_bus->write_timeout(BNO085_ADDR, reset_pkt, 5) == 5) {
            sleep_ms(300);
            return 0;
        }
        sleep_ms(30);
    }
    return -1;
}

void BNO085::hal_close(sh2_Hal_t* self) {
    // Nothing to do
}

int BNO085::hal_read(sh2_Hal_t* self, uint8_t* buf, unsigned len, uint32_t* t_us) {

    #if DEBUG_I2C_TIMING
        printf("Starting BNO Read....\n");
        uint32_t start = time_us_32();
        uint32_t stop = start;
    #endif


    uint8_t header[4];
    if (g_instance->_bus->read_timeout(BNO085_ADDR, header, 4) != 4) {
        return 0;
    }
    
    #if DEBUG_I2C_TIMING
        stop = time_us_32();
        printf("\tRead Header Time: %d (%d -> %d)\n", (stop - start), start, stop);
    #endif

    // Get packet size
    uint16_t packet_size = header[0] | (header[1] << 8);
    packet_size &= ~0x8000; // Clear continue bit
    
    if (packet_size == 0 || packet_size > len) {
        return 0;
    }
    
    // Read remaining data
    uint16_t remaining = packet_size;
    uint8_t* ptr = buf;
    
    // First read includes header
    uint16_t read_size = (remaining > 32) ? 32 : remaining;
    uint8_t temp[64];
    
    if (g_instance->_bus->read_timeout(BNO085_ADDR, temp, read_size) != read_size) {
        return 0;
    }

    memcpy(ptr, temp, read_size);
    ptr += read_size;
    remaining -= read_size;

    #if DEBUG_I2C_TIMING
        stop = time_us_32();
        printf("\tMain Body Read Time: %d (%d -> %d)\n", (stop - start), start, stop);
    #endif

    // Read rest if needed
    while (remaining > 0) {
        read_size = (remaining > 28) ? 28 : remaining;
        
        if (g_instance->_bus->read_timeout(BNO085_ADDR, temp, read_size + 4) != (read_size + 4)) {
            return 0;
        }
        // Skip header in subsequent reads
        memcpy(ptr, temp + 4, read_size);
        ptr += read_size;
        remaining -= read_size;

        #if DEBUG_I2C_TIMING
            stop = time_us_32();
            printf("\t\tExtra Body Read Time: %d (%d -> %d)\n", (stop - start), start, stop);
        #endif
    }
    
    #if DEBUG_I2C_TIMING
        stop = time_us_32();
        printf("\tRead Time: %d (%d -> %d)\n", (stop - start), start, stop);
        printf("=====================================================\n");
    #endif
    return packet_size;
}

int BNO085::hal_write(sh2_Hal_t* self, uint8_t* buf, unsigned len) {
    uint16_t write_size = (len > 32) ? 32 : len;
    
    if (g_instance->_bus->write_timeout(BNO085_ADDR, buf, write_size) != write_size) {
        return 0;
    }
    
    return write_size;
}

uint32_t BNO085::hal_get_time_us(sh2_Hal_t* self) {
    return to_us_since_boot(get_absolute_time());
}

void BNO085::hal_callback(void* cookie, sh2_AsyncEvent_t* event) {
    // Handle reset events if needed
}

void BNO085::sensor_handler(void* cookie, sh2_SensorEvent_t* event) {
    BNO085* instance = static_cast<BNO085*>(cookie);
    
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        return;
    }
    
    // Update data based on sensor type
    switch (value.sensorId) {
        case SH2_ACCELEROMETER:
            instance->_data.accel_x = value.un.accelerometer.x;
            instance->_data.accel_y = value.un.accelerometer.y;
            instance->_data.accel_z = value.un.accelerometer.z;
            instance->_data.valid = true;
            instance->_data_ready = true;
            sleep_ms(1);
            break;
    }
}

} // namespace drivers