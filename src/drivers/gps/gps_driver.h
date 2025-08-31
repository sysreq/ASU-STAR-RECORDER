#pragma once

// Project Omni-Header
#include "config/all_headers.h"

namespace drivers {

struct GpsData {
    bool valid = false;
    uint32_t unix_time = 0;     // Unix epoch seconds
    double lon = 0;             // degrees
    double lat = 0;             // degrees
    int32_t height = 0;         // mm above ellipsoid
    int32_t hMSL = 0;           // mm above mean sea level
    uint32_t hAcc = 0;          // mm horizontal accuracy
    uint32_t vAcc = 0;          // mm vertical accuracy
    int32_t velN = 0;           // mm/s north
    int32_t velE = 0;           // mm/s east
    int32_t velD = 0;           // mm/s down
    int32_t gSpeed = 0;         // mm/s ground speed
    float heading = 0;          // degrees * 1e5
    uint32_t sAcc = 0;          // mm/s speed accuracy
    uint32_t headingAcc = 0;    // degrees * 1e5 heading accuracy
};

class GpsDriver {
private:
    static constexpr size_t BUFFER_SIZE = 512;
    uart_inst_t* uart = nullptr;
    uint8_t buffer[BUFFER_SIZE];
    size_t buf_pos = 0;
    GpsData data;
    bool use_ubx = false;
    
    // NMEA parsing state
    char nmea_line[256];
    size_t nmea_pos = 0;
    
    // Parsing functions
    bool parse_nav_pvt(const uint8_t* payload, size_t len);
    bool parse_nmea_sentence(const char* sentence);
    bool parse_gga(const char* sentence);
    bool parse_rmc(const char* sentence);
    bool parse_vtg(const char* sentence);
    
public:
    bool init(uart_inst_t* u, uint rx_pin, uint tx_pin, bool ubx_protocol = true);
    bool update();
    const GpsData& get_data() const { return data; }
    void clear() { data.valid = false; }
    void reset() { data = GpsData(); }
    void set_led_enabled(bool enabled);
};

} // namespace drivers