#pragma once

// Project Omni-Header
#include "config/all_headers.h"

#include "pico/rand.h"

namespace drivers {

class Display {
private:
    static constexpr uint8_t SH1107_ADDR = 0x3C;
    static constexpr size_t WIDTH = 128;
    static constexpr size_t HEIGHT = 64;
    
    uint8_t buffer[32][64] = {0};
    i2c_inst_t* i2c = nullptr;

    static constexpr uint8_t font[][5] = {
        {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
        {0x7F, 0x49, 0x49, 0x49, 0x41}, // e -> using E pattern
        {0x00, 0x41, 0x7F, 0x41, 0x00}, // l -> using I pattern (simplified)
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, // o -> using O pattern
    };

    void write_cmd(uint8_t cmd) {
        uint8_t buf[2] = {0x00, cmd};
        i2c_write_blocking(i2c, SH1107_ADDR, buf, 2, false);
    }

    void write_data(const uint8_t* data, size_t len) {
        uint8_t buf[129];
        buf[0] = 0x40;  // Data mode
        for (size_t i = 0; i < len && i < 128; i++) {
            buf[i + 1] = data[i];
        }
        i2c_write_blocking(i2c, SH1107_ADDR, buf, len + 1, false);
    }

    void set_pixel(uint8_t x, uint8_t y, bool on = true) {
        if (x >= WIDTH || y >= HEIGHT) return;
        
        uint8_t page = y / 8;
        uint8_t bit = y % 8;
        
        if (on) {
            buffer[page][x] |= (1 << bit);
        } else {
            buffer[page][x] &= ~(1 << bit);
        }
    }

    // Draw a horizontal line
    void draw_horizontal_line(uint8_t y, uint8_t x_start = 0, uint8_t x_end = WIDTH - 1) {
        if (y >= HEIGHT) return;
        
        for (uint8_t x = x_start; x <= x_end && x < WIDTH; x++) {
            set_pixel(y, x, true);
        }
    }

    // Draw a vertical line
    void draw_vertical_line(uint8_t x, uint8_t y_start = 0, uint8_t y_end = HEIGHT - 1) {
        if (x >= WIDTH) return;
        
        for (uint8_t y = y_start; y <= y_end && y < HEIGHT; y++) {
            set_pixel(y, x, true);
        }
    }

    // Clear the buffer
    void clear() {
        memset(buffer, 0, sizeof(buffer));
    }

    // Update the display with buffer contents
    void update() {

        for (uint8_t page = 0; page < 32; page++) {

            write_cmd(0xB0 | page);
            write_cmd(0x00);  // Lower column address
            write_cmd(0x10);  // Upper column address

            buffer[page][0] = get_rand_32() & 0xFF;
            // Set column address to 0
            // For 128x64 display centered in 128x128 controller            
            // Write the entire page (128 bytes)
            write_data(buffer[page], 64);
        }
    }

public:
    bool init(i2c_inst_t* i2c_port, uint sda_pin, uint scl_pin, uint baudrate = 400000) {
        // Initialize I2C (default pins for Pico: GP4=SDA, GP5=SCL)
        i2c_init(i2c_port, baudrate);
        gpio_set_function(sda_pin, GPIO_FUNC_I2C);
        gpio_set_function(scl_pin, GPIO_FUNC_I2C);
        gpio_pull_up(sda_pin);
        gpio_pull_up(scl_pin);
        
        sleep_ms(100);  // Give display time to power up
        this->i2c = i2c_port;
        
        // Proper initialization sequence for SH1107
        write_cmd(0xAE);  // Display off
        write_cmd(0xD5);  // Set display clock divide
        write_cmd(0x51);
        write_cmd(0x21);  // Memory mode
        write_cmd(0x81);  // Set contrast
        write_cmd(0x4F);
        write_cmd(0xAD);  // DC-DC control
        write_cmd(0x8A);  // DC-DC on
        write_cmd(0xA0);  // Segment remap (A1 for rotated)
        write_cmd(0xC0);  // COM scan direction (C8 for rotated)
        write_cmd(0xDC);  // Set display start line
        write_cmd(0x00);
        write_cmd(0xD3);  // Display offset
        write_cmd(0x60);  // Offset to center 64x128 in 128x128 buffer
        write_cmd(0xD9);  // Set precharge
        write_cmd(0x22);
        write_cmd(0xDB);  // VCOMH deselect
        write_cmd(0x35);
        write_cmd(0xA8);  // Set multiplex
        write_cmd(0x3F);  // 64 MUX
        write_cmd(0xA4);  // Display all on resume
        write_cmd(0xA6);  // Normal display
        sleep_ms(100);  // Delay before turning on
        write_cmd(0xAF);  // Display on

        printf("[Display][OK] Initialized successfully.\n");

        return true;
    }

    void power_on() {
        clear();
        //buffer[12][63] = 0xFF;
        update();
    }
};


}