#pragma once

// Project Omni-Header
#include "config/all_headers.h"

namespace drivers::utils {

// Verify UBX checksum
inline bool verify_ubx_checksum(const uint8_t* msg, size_t len) {
    if (len < 8) return false;
    
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < len - 2; ++i) {
        ck_a += msg[i];
        ck_b += ck_a;
    }
    
    return (ck_a == msg[len - 2]) && (ck_b == msg[len - 1]);
}

// Verify NMEA checksum
inline bool verify_nmea_checksum(const char* sentence) {
    if (!sentence || sentence[0] != '$') return false;
    
    const char* asterisk = std::strchr(sentence, '*');
    if (!asterisk || std::strlen(asterisk) < 3) return false;
    
    uint8_t calculated = 0;
    for (const char* p = sentence + 1; p < asterisk; ++p) {
        calculated ^= static_cast<uint8_t>(*p);
    }
    
    auto hex_to_val = [](char c) -> uint8_t {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return 0;
    };
    
    uint8_t provided = (hex_to_val(asterisk[1]) << 4) | hex_to_val(asterisk[2]);
    return calculated == provided;
}

// Extract NMEA field
inline bool extract_nmea_field(const char* sentence, uint8_t field_num, 
                               char* buffer, size_t buffer_size) {
    if (!sentence || !buffer || buffer_size == 0) return false;
    
    const char* start = sentence;
    uint8_t current_field = 0;
    
    while (current_field < field_num) {
        start = std::strchr(start, ',');
        if (!start) return false;
        start++;
        current_field++;
    }
    
    const char* end = std::strchr(start, ',');
    if (!end) {
        end = std::strchr(start, '*');
        if (!end) return false;
    }
    
    size_t len = end - start;
    if (len >= buffer_size) len = buffer_size - 1;
    
    std::memcpy(buffer, start, len);
    buffer[len] = '\0';
    
    return true;
}

// Parse NMEA coordinate (ddmm.mmmm format)
inline double parse_nmea_coordinate(const char* coord, char dir) {
    if (!coord || std::strlen(coord) < 3) return 0.0;
    
    double value = std::atof(coord);
    int32_t degrees = static_cast<int32_t>(value / 100.0);
    double minutes = value - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    
    if (dir == 'S' || dir == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

// Convert GPS time to Unix time
inline uint32_t gps_to_unix_time(uint16_t year, uint8_t month, uint8_t day, 
                                 uint8_t hour, uint8_t min, uint8_t sec) {
    // Days since Unix epoch for each month (non-leap year)
    static const uint16_t days[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    
    // Calculate days since 1970
    uint32_t total_days = (year - 1970) * 365;
    
    // Add leap days
    total_days += (year - 1969) / 4;
    total_days -= (year - 1901) / 100;
    total_days += (year - 1601) / 400;
    
    // Add days for current year
    total_days += days[month - 1];
    total_days += day - 1;
    
    // Add leap day for current year if applicable
    if (month > 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
        total_days++;
    }
    
    // Convert to seconds
    return total_days * 86400UL + hour * 3600UL + min * 60UL + sec;
}

} // namespace gps_utils