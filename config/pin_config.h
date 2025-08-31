#pragma once

#include "all_headers.h"

// UART0 - GPS NEO6M
constexpr uint8_t GPS_UART_RX = 12;
constexpr uint8_t GPS_UART_TX = 13;


// SPI0 - SD Card
constexpr uint8_t SD_SPI_MISO = 16;
constexpr uint8_t SD_SPI_MOSI = 19;
constexpr uint8_t SD_SPI_SCK = 18;
constexpr uint8_t SD_SPI_CS = 17;

// SPI1 - HX711 Force Sensor (bit-banged)
constexpr uint8_t HX711_DATA = 12;
constexpr uint8_t HX711_SCK = 13;

// I2C0 - Sensor Hub (5-port passive hub)
constexpr uint8_t I2C0_SDA = 4;
constexpr uint8_t I2C0_SCL = 5;

// I2C1 - Display
constexpr uint8_t I2C1_SDA = 6;
constexpr uint8_t I2C1_SCL = 7;

// System Status
constexpr uint8_t LED_STATUS = 25;
constexpr uint8_t BTN_MODE = 20;