#pragma once

#include <stdio.h>

// C++23 Core language and traits
#include <cstddef>      // size_t, ptrdiff_t, nullptr_t
#include <cstdint>      // fixed-width integer types
#include <cstring>
#include <type_traits>  // type traits, enable_if, conditional, decay, etc.
#include <concepts>     // concepts for templates and constraints
#include <optional>

// C++23 Containers
#include <array>

// C++23 Utilities
#include <utility>      // move, forward, exchange, pair
#include <tuple>        // tuple, tie, apply
#include <algorithm>    // general-purpose tools - min, max, clamp
#include <functional>   // reusable tools for working with functions
#include <cmath>
#include <cstdarg>

// C++23 Views
#include <span>         // std::span for non-owning views
#include <string_view>

// C++23 Common, low-cost additions likely useful across the library
#include <atomic>
#include <limits>       // std::numeric_limits
#include <compare>      // three-way comparison (C++20/23)
#include <bit>          // bit_cast, has_single_bit, bit_width
#include <initializer_list>
#include <expected>

// C Types
#include <inttypes.h>

// ===========================================================================================
// Pi Pico SDK
// All files will be depricated in time.
// ===========================================================================================

// Run-Time Libraries
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <pico/bootrom.h>
#include <pico/time.h>

// Low-Level Hardware Interfaces
#include <hardware/resets.h>
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <hardware/i2c.h>
#include <hardware/gpio.h>
#include <hardware/uart.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/timer.h>
#include <hardware/pwm.h>

namespace utils {
    // 16-bit version (2 bytes)
    template<typename T>
    static inline constexpr T merge_bytes(uint8_t msb, uint8_t lsb) {
        static_assert(std::is_same_v<T, int16_t> || std::is_same_v<T, uint16_t>, 
                    "T must be int16_t or uint16_t");
        
        T raw = (static_cast<T>(msb) << 8) | static_cast<T>(lsb);
        
        // No sign extension needed for 16-bit as casting handles it
        return raw;
    }

    // Sign extend for signed types
    template<typename T>
    static inline constexpr T merge_bytes(uint8_t msb, uint8_t mid, uint8_t lsb) {
        static_assert(std::is_same_v<T, int32_t> || std::is_same_v<T, uint32_t>, 
                    "T must be int32_t or uint32_t");
        
        T raw = (static_cast<T>(msb) << 16) | 
                (static_cast<T>(mid) << 8) | 
                static_cast<T>(lsb);
        
        // Sign extend for signed types
        if constexpr (std::is_signed_v<T>) {
            if (raw & 0x800000) {
                raw |= 0xFF000000;
            }
        }
        
        return raw;
    }

    template<typename Tag>
    class Timer {
    private:
        static inline uint32_t total_time = 0;
        static inline size_t count = 0;

        uint32_t start;
        
    public:
        Timer() : start(time_us_32()) {}
        
        ~Timer() {
            auto end = time_us_32();
            auto duration = end - start;
            total_time += duration;
            count++;
        }
        
        static double getAverage() {
            return count > 0 ? total_time / (count * 1.0) : 0.0;
        }
    };

    static inline constexpr uint32_t hz_to_us(const uint32_t freq) {
        return 1000000U / freq;
    }

    static inline constexpr uint32_t hz_to_ms(const uint32_t freq) {
        return 1000U / freq;
    }
}