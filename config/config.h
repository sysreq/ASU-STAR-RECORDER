#pragma once

#include "all_headers.h"

// ============================================
// SYSTEM CONFIGURATION
// ============================================

namespace config {

// ============================================
// CORE SYSTEM SETTINGS
// ============================================
namespace system {
    static constexpr uint32_t CLOCK_HZ = 150000000;           // 150MHz for both cores
    static constexpr uint TOGGLE_PIN = 14;
    static constexpr uint BUTTON_PIN = 15;
}

// ============================================
// PIN ASSIGNMENTS
// ============================================
namespace pins {    
    // SPI0 - SD Card        
    // System Status (currently unused)
    namespace status {

    }
    
    namespace hx711 {
        // static constexpr uint8_t DATA = 14; 
        // static constexpr uint8_t SCK = 15;
    }
}

// ============================================
// SD CARD CONFIGURATION
// ============================================
namespace sdcard {
    static spi_inst_t*  SPI_BUS = spi0;
    static constexpr uint8_t MISO = 16;
    static constexpr uint8_t CS   = 17;
    static constexpr uint8_t SCK  = 18;
    static constexpr uint8_t MOSI = 19;

    static constexpr uint32_t FREQ_HZ = 31250000;       // 31.25 MHz (125MHz/4)
    static constexpr size_t FILE_BUFFER_SIZE = 512;           // Per-file buffer size


}

// ============================================
// I2C CONFIGURATION
// ============================================
namespace i2c {
    namespace bus0 {
        static constexpr uint8_t SDA = 4;
        static constexpr uint8_t SCL = 5;
        static constexpr uint32_t DATA_RATE = 1'000'000;          // 400kHz for sensors
    }

    namespace bus1 {
        static constexpr uint8_t SDA = 6;
        static constexpr uint8_t SCL = 7;
        static constexpr uint32_t FREQ_HZ = 400'000;          // 400kHz for sensors
    }

    
    // Device addresses on I2C0
    namespace addresses {
        static constexpr uint8_t BMP390_ADDR = 0x77;               // Barometric pressure sensor (active)
        static constexpr uint8_t BMP581_ADDR = 0x47;
        static constexpr uint8_t BNO085_ADDR = 0x4A;               // IMU sensor (active)
        static constexpr uint8_t HX711  = 0x2A;                    // Forceplate Sensor (active)
        static constexpr uint8_t ICM20948_ADDR = 0x69;             // IMU sensor (active)

        // Future/alternative devices (not currently used)
        static constexpr uint8_t BME280 = 0x76;               // Alternative barometric sensor
        static constexpr uint8_t MPU6050_1 = 0x68;            // Alternative IMU #1
        static constexpr uint8_t MPU6050_2 = 0x69;            // Alternative IMU #2
        static constexpr uint8_t PITOT = 0x28;                // Pitot tube sensor
        static constexpr uint8_t SH1107 = 0x3C;               // Display controller (I2C1)
    }
}

// ============================================
// GPS CONFIGURATION
// ============================================
namespace gps {
    static constexpr uint8_t RX_PIN = 12;
    static constexpr uint8_t TX_PIN = 13;

    // Protocol settings
    static constexpr bool USE_BINARY_UBX = false;              // true = UBX binary, false = NMEA text
    static constexpr uint8_t UPDATE_RATE_HZ = 1;              // Updates per second (max 5)
    static constexpr uint32_t BAUD_RATE = 115200;             // Target baud rate (starts at 9600)
    
    // Data streams to enable
    static constexpr bool ENABLE_POSITION = true;             // Position data
    static constexpr bool ENABLE_VELOCITY = true;             // Velocity data
    static constexpr bool ENABLE_TIME = true;                 // Time data
    static constexpr bool ENABLE_SATELLITES = false;          // Satellite info
    static constexpr bool ENABLE_STATUS = false;              // Navigation status
    
    // Configuration method
    static constexpr bool POLL_CONFIG = true;                 // true = smart config, false = force config
    
    // Buffer size
    static constexpr size_t BUFFER_SIZE = 512;                // GPS data buffer
}

// ============================================
// SENSOR SAMPLING RATES
// ============================================
namespace sensors {
    static constexpr uint32_t RAW_DATA_HZ = 10;
    static constexpr uint32_t BNO_RATE_HZ = 10;              // BNO085, MPU6050
    static constexpr uint32_t IMU_RATE_HZ = 10;                 //BNO085
    static constexpr uint32_t GPS_RATE_HZ = 1;                  // NEO6M
    static constexpr uint32_t BARO_RATE_HZ = 10;                // BMP390, BME280
    static constexpr uint32_t PITOT_RATE_HZ = 20;               // Pitot tube
    static constexpr uint32_t FORCE_RATE_HZ = 20;               // HX711
    static constexpr uint32_t LOG_FLUSH_RATE_HZ = 2;            // SD card flush
}

namespace icm20948 {
    // Accelerometer range: 0=±2g, 1=±4g, 2=±8g, 3=±16g
    static constexpr uint8_t ACCEL_RANGE = 1;  // ±4g
    
    // Gyroscope range: 0=±250dps, 1=±500dps, 2=±1000dps, 3=±2000dps
    static constexpr uint8_t GYRO_RANGE = 1;   // ±500dps
    
    // Compile-time scale factor calculations
    static constexpr float ACCEL_SCALE = (ACCEL_RANGE == 0) ? (9.81f / 16384.0f) :
                                            (ACCEL_RANGE == 1) ? (9.81f / 8192.0f) :
                                            (ACCEL_RANGE == 2) ? (9.81f / 4096.0f) :
                                                                (9.81f / 2048.0f);
    
    static constexpr float DEG_TO_RAD = 0.017453293f;
    static constexpr float GYRO_SCALE = (GYRO_RANGE == 0) ? (250.0f / 32768.0f * DEG_TO_RAD) :
                                        (GYRO_RANGE == 1) ? (500.0f / 32768.0f * DEG_TO_RAD) :
                                        (GYRO_RANGE == 2) ? (1000.0f / 32768.0f * DEG_TO_RAD) :
                                                            (2000.0f / 32768.0f * DEG_TO_RAD);
}

namespace pitot_tube {
    static constexpr float PRESSURE_RANGE_PSI = 1.0f;  // Differential pressure range
    static constexpr int CALIBRATION_SAMPLES = 50;     // Samples for zero calibration
    static constexpr float PSI_TO_PA = 6894.76f;
    static constexpr float MS_TO_MPH = 2.237f;
    static constexpr float STANDARD_AIR_DENSITY = 1.225f;  // kg/m³ at sea level
}

// ============================================
// DISPLAY CONFIGURATION
// ============================================
namespace display {
    static constexpr uint32_t UPDATE_RATE_HZ = 10;            // 10Hz when active
    static constexpr bool OFF_DURING_FLIGHT = true;           // Power off during flight
}
} // namespace config

// ============================================
// LEGACY COMPATIBILITY DEFINES
// ============================================
// These maintain backward compatibility with existing code
// TODO: Update code to use config:: namespace directly

// Pin assignments
#define GPS_UART_RX config::pins::gps::RX
#define GPS_UART_TX config::pins::gps::TX
#define SD_SPI_MISO config::pins::sdcard::MISO
#define SD_SPI_CS   config::pins::sdcard::CS
#define SD_SPI_SCK  config::pins::sdcard::SCK
#define SD_SPI_MOSI config::pins::sdcard::MOSI
#define I2C0_SDA    config::pins::i2c0::SDA
#define I2C0_SCL    config::pins::i2c0::SCL
#define I2C1_SDA    config::pins::i2c1::SDA
#define I2C1_SCL    config::pins::i2c1::SCL

// System settings
#define SYS_CLOCK_HZ     config::system::CLOCK_HZ
#define GPS_BUFFER_SIZE  config::gps::BUFFER_SIZE
#define I2C0_FREQ_HZ     config::i2c::BUS0_FREQ_HZ
#define I2C1_FREQ_HZ     config::i2c::BUS1_FREQ_HZ