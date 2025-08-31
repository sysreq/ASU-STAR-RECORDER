#pragma once

#include "config/all_headers.h"

// Core Clock Settings
constexpr uint32_t SYS_CLOCK_HZ = 150000000;  // 150MHz for both cores

namespace config::gps {
    // GPS Protocol Selection
    constexpr uint8_t GPS_UPDATE_RATE_HZ = 1;        // Updates per second. Max 5.
    constexpr uint32_t GPS_BAUD_RATE = 115200;       // Data transfer rate (starts and 9600 and goes to this)

    constexpr bool GPS_USE_BINARY_UBX = true;       // true = UBX binary (faster), false = NMEA text (easier to read)

    constexpr bool GPS_ENABLE_POSITION = true;      // Position data (NMEA: GGA/GLL, UBX: POSLLH)
    constexpr bool GPS_ENABLE_VELOCITY = true;      // Velocity data (NMEA: RMC/VTG, UBX: VELNED)
    constexpr bool GPS_ENABLE_TIME = true;          // Time data (NMEA: ZDA/RMC, UBX: included in PVT)
    constexpr bool GPS_ENABLE_SATELLITES = false;   // Satellite info (NMEA: GSV/GSA, UBX: SAT/DOP)
    constexpr bool GPS_ENABLE_STATUS = false;       // Navigation status (NMEA: GSA, UBX: STATUS)

    
    constexpr bool GPS_POLL_CONFIG = true;          // true = change settings based on existing ones (faster)
                                                    // false = changes settings regardless of current state (2+ seconds required)
};

// Sampling Rates (Hz)
constexpr uint32_t IMU_SAMPLE_RATE = 100;     // BNO085, MPU6050
constexpr uint32_t GPS_SAMPLE_RATE = 10;      // NEO6M
constexpr uint32_t BARO_SAMPLE_RATE = 25;     // BME280
constexpr uint32_t PITOT_SAMPLE_RATE = 50;    // Pitot tube
constexpr uint32_t FORCE_SAMPLE_RATE = 80;    // HX711
constexpr uint32_t LOG_FLUSH_RATE = 2;        // SD card flush

// Display Settings
constexpr uint32_t DISPLAY_UPDATE_RATE = 10;  // 10Hz when active
constexpr bool DISPLAY_OFF_DURING_FLIGHT = true;

// Buffer Sizes
constexpr size_t GPS_BUFFER_SIZE = 512;
constexpr size_t LOG_BUFFER_SIZE = 8192;      // 8KB write buffer
constexpr size_t FIFO_QUEUE_SIZE = 64;        // Inter-core queue

// I2C Addresses (via hub on I2C0)
constexpr uint8_t BNO085_ADDR = 0x4A;
constexpr uint8_t BME280_ADDR = 0x76;
constexpr uint8_t MPU6050_1_ADDR = 0x68;
constexpr uint8_t MPU6050_2_ADDR = 0x69;
constexpr uint8_t PITOT_ADDR = 0x28;

// I2C1 Display Address
constexpr uint8_t SH1107_ADDR = 0x3C;

// I2C Speeds
constexpr uint32_t I2C0_FREQ_HZ = 400000;     // 400kHz for sensors
constexpr uint32_t I2C1_FREQ_HZ = 400000;     // 400kHz for display