#include "gps_driver.h"
#include "gps_utils.h"

namespace drivers {

bool GpsDriver::init(uart_inst_t* u, uint rx_pin, uint tx_pin, bool ubx_protocol) {
    uart = u;
    use_ubx = ubx_protocol;
    
    // Initialize UART
    uart_init(uart, 9600);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    
    sleep_ms(100);
    
    // Common disable messages
    uint8_t disable_gll[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x01,0x00,0xFB,0x11};
    uint8_t disable_gsa[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x02,0x00,0xFC,0x13};
    uint8_t disable_gsv[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x03,0x00,0xFD,0x15};
    
    if (use_ubx) {
        // Disable all NMEA
        uint8_t disable_gga[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x00,0xFA,0x0F};
        uint8_t disable_rmc[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x00,0xFE,0x17};
        uint8_t disable_vtg[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x00,0xFF,0x19};
        uint8_t enable_pvt[]  = {0xB5,0x62,0x06,0x01,0x03,0x00,0x01,0x07,0x01,0x13,0x51};
        
        uart_write_blocking(uart, disable_gga, sizeof(disable_gga));
        sleep_ms(10);
        uart_write_blocking(uart, disable_gll, sizeof(disable_gll));
        sleep_ms(10);
        uart_write_blocking(uart, disable_gsa, sizeof(disable_gsa));
        sleep_ms(10);
        uart_write_blocking(uart, disable_gsv, sizeof(disable_gsv));
        sleep_ms(10);
        uart_write_blocking(uart, disable_rmc, sizeof(disable_rmc));
        sleep_ms(10);
        uart_write_blocking(uart, disable_vtg, sizeof(disable_vtg));
        sleep_ms(10);
        uart_write_blocking(uart, enable_pvt, sizeof(enable_pvt));
        sleep_ms(10);
    } else {
        // Enable only GGA, RMC, VTG for NMEA
        uint8_t enable_gga[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x00,0x01,0xFB,0x10};
        uint8_t enable_rmc[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x04,0x01,0xFF,0x18};
        uint8_t enable_vtg[] = {0xB5,0x62,0x06,0x01,0x03,0x00,0xF0,0x05,0x01,0x00,0x1A};
        
        // Disable unwanted
        uart_write_blocking(uart, disable_gll, sizeof(disable_gll));
        sleep_ms(10);
        uart_write_blocking(uart, disable_gsa, sizeof(disable_gsa));
        sleep_ms(10);
        uart_write_blocking(uart, disable_gsv, sizeof(disable_gsv));
        sleep_ms(10);
        
        // Enable wanted
        uart_write_blocking(uart, enable_gga, sizeof(enable_gga));
        sleep_ms(10);
        uart_write_blocking(uart, enable_rmc, sizeof(enable_rmc));
        sleep_ms(10);
        uart_write_blocking(uart, enable_vtg, sizeof(enable_vtg));
        sleep_ms(10);
    }

    sleep_ms(10);
    set_led_enabled(false);
    sleep_ms(10);

    printf("GPS initialized.\n");
    return true;
}

bool GpsDriver::update() {
    while (uart_is_readable(uart)) {
        uint8_t byte = uart_getc(uart);
        
        if (use_ubx) {
            // UBX protocol parsing
            if (buf_pos == 0 && byte != 0xB5) continue;
            if (buf_pos == 1 && byte != 0x62) { buf_pos = 0; continue; }
            
            buffer[buf_pos++] = byte;
            
            if (buf_pos >= 6) {
                uint16_t len = buffer[4] | (buffer[5] << 8);
                if (buf_pos >= len + 8) {
                    // Check message class and ID for NAV-PVT
                    if (buffer[2] == 0x01 && buffer[3] == 0x07) {
                        if (utils::verify_ubx_checksum(buffer, len + 8)) {
                            parse_nav_pvt(buffer + 6, len);
                        }
                    }
                    buf_pos = 0;
                }
            }
            
            if (buf_pos >= BUFFER_SIZE) buf_pos = 0;
        } else {
            // NMEA protocol parsing
            if (byte == '$') {
                nmea_pos = 0;
            }
            
            if (nmea_pos < sizeof(nmea_line) - 1) {
                nmea_line[nmea_pos++] = byte;
                
                if (byte == '\n') {
                    nmea_line[nmea_pos] = '\0';
                    printf(nmea_line);
                    parse_nmea_sentence(nmea_line);
                    nmea_pos = 0;
                }
            } else {
                nmea_pos = 0;
            }
        }
    }
    
    return data.valid;
}

bool GpsDriver::parse_nav_pvt(const uint8_t* payload, size_t len) {
    if (len < 92) return false;  // NAV-PVT is 92 bytes
    
    // Direct field extraction from NAV-PVT message
    uint16_t year = payload[4] | (payload[5] << 8);
    uint8_t month = payload[6];
    uint8_t day = payload[7];
    uint8_t hour = payload[8];
    uint8_t min = payload[9];
    uint8_t sec = payload[10];
    
    uint8_t fix_type = payload[20];
    uint8_t flags = payload[21];
    
    data.lon = *reinterpret_cast<const int32_t*>(payload + 24) / 1e7;
    data.lat = *reinterpret_cast<const int32_t*>(payload + 28) / 1e7;
    data.height = *reinterpret_cast<const int32_t*>(payload + 32);
    data.hMSL = *reinterpret_cast<const int32_t*>(payload + 36);
    data.hAcc = *reinterpret_cast<const uint32_t*>(payload + 40);
    data.vAcc = *reinterpret_cast<const uint32_t*>(payload + 44);
    data.velN = *reinterpret_cast<const int32_t*>(payload + 48);
    data.velE = *reinterpret_cast<const int32_t*>(payload + 52);
    data.velD = *reinterpret_cast<const int32_t*>(payload + 56);
    data.gSpeed = *reinterpret_cast<const int32_t*>(payload + 60);
    data.heading = *reinterpret_cast<const int32_t*>(payload + 64);
    data.sAcc = *reinterpret_cast<const uint32_t*>(payload + 68);
    data.headingAcc = *reinterpret_cast<const uint32_t*>(payload + 72);
    
    // Convert GPS time to Unix time
    data.unix_time = utils::gps_to_unix_time(year, month, day, hour, min, sec);
    
    // Check fix validity
    data.valid = (fix_type >= 2) && (flags & 0x01);
    
    return true;
}

bool GpsDriver::parse_nmea_sentence(const char* sentence) {
    if (!sentence || sentence[0] != '$') return false;
    
    // Verify checksum
    if (!utils::verify_nmea_checksum(sentence)) return false;
    
    if (std::strncmp(sentence, "$GPGGA", 6) == 0 || std::strncmp(sentence, "$GNGGA", 6) == 0) {
        return parse_gga(sentence);
    } else if (std::strncmp(sentence, "$GPRMC", 6) == 0 || std::strncmp(sentence, "$GNRMC", 6) == 0) {
        return parse_rmc(sentence);
    } else if (std::strncmp(sentence, "$GPVTG", 6) == 0 || std::strncmp(sentence, "$GNVTG", 6) == 0) {
        return parse_vtg(sentence);
    }
    
    return false;
}

bool GpsDriver::parse_gga(const char* sentence) {
    char field[32];
    
    // Field 2,3: Latitude
    if (utils::extract_nmea_field(sentence, 2, field, sizeof(field)) && field[0]) {
        char dir[2];
        if (utils::extract_nmea_field(sentence, 3, dir, sizeof(dir))) {
            data.lat = utils::parse_nmea_coordinate(field, dir[0]);
        }
    }
    
    // Field 4,5: Longitude
    if (utils::extract_nmea_field(sentence, 4, field, sizeof(field)) && field[0]) {
        char dir[2];
        if (utils::extract_nmea_field(sentence, 5, dir, sizeof(dir))) {
            data.lon = utils::parse_nmea_coordinate(field, dir[0]);
        }
    }
    
    // Field 6: Fix quality
    if (utils::extract_nmea_field(sentence, 6, field, sizeof(field))) {
        int fix = std::atoi(field);
        data.valid = (fix >= 1);
    }
    
    // Field 9,10: Altitude (MSL)
    if (utils::extract_nmea_field(sentence, 9, field, sizeof(field)) && field[0]) {
        double alt = std::atof(field);
        data.hMSL = static_cast<int32_t>(alt * 1000);
        data.height = data.hMSL;  // Approximation
    }
    
    return true;
}

bool GpsDriver::parse_rmc(const char* sentence) {
    char field[32];
    
    // Field 1: Time (HHMMSS.sss)
    uint8_t hour = 0, min = 0, sec = 0;
    if (utils::extract_nmea_field(sentence, 1, field, sizeof(field)) && field[0]) {
        hour = (field[0] - '0') * 10 + (field[1] - '0');
        min = (field[2] - '0') * 10 + (field[3] - '0');
        sec = (field[4] - '0') * 10 + (field[5] - '0');
    }
    
    // Field 9: Date (DDMMYY)
    uint8_t day = 0, month = 0;
    uint16_t year = 0;
    if (utils::extract_nmea_field(sentence, 9, field, sizeof(field)) && field[0]) {
        day = (field[0] - '0') * 10 + (field[1] - '0');
        month = (field[2] - '0') * 10 + (field[3] - '0');
        year = 2000 + (field[4] - '0') * 10 + (field[5] - '0');
        
        data.unix_time = utils::gps_to_unix_time(year, month, day, hour, min, sec);
    }
    
    // Field 7: Speed in knots
    if (utils::extract_nmea_field(sentence, 7, field, sizeof(field)) && field[0]) {
        double knots = std::atof(field);
        data.gSpeed = static_cast<int32_t>(knots * 514.444);  // knots to mm/s
    }
    
    // Field 8: Course
    if (utils::extract_nmea_field(sentence, 8, field, sizeof(field)) && field[0]) {
        double course = std::atof(field);
        data.heading = static_cast<int32_t>(course * 1e5);
    }
    
    return true;
}

bool GpsDriver::parse_vtg(const char* sentence) {
    char field[32];
    
    // Field 1: Course
    if (utils::extract_nmea_field(sentence, 1, field, sizeof(field)) && field[0]) {
        double course = std::atof(field);
        data.heading = static_cast<int32_t>(course * 1e5);
    }
    
    // Field 7: Speed in km/h
    if (utils::extract_nmea_field(sentence, 7, field, sizeof(field)) && field[0]) {
        double kmh = std::atof(field);
        data.gSpeed = static_cast<int32_t>(kmh * 277.778);  // km/h to mm/s
    }
    
    return true;
}

void GpsDriver::set_led_enabled(bool enabled) {

    uint8_t cfg_tp5_enable[] = {
        0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x42,
        0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA0, 0x86, 
        0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x11, 0x0E
    };

    uint8_t cfg_tp5_disable[] = {
        0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
        0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x99, 0x7B
    };
    
    uint8_t cfg_tp5[40] = {};
    memcpy(cfg_tp5, (enabled ? cfg_tp5_enable : cfg_tp5_disable), 40);

    // Calculate UBX checksum
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < sizeof(cfg_tp5) - 2; i++) {
        ck_a += cfg_tp5[i];
        ck_b += ck_a;
    }
    cfg_tp5[sizeof(cfg_tp5) - 2] = ck_a;
    cfg_tp5[sizeof(cfg_tp5) - 1] = ck_b;
    
    // Send the configuration message
    uart_write_blocking(uart, cfg_tp5, sizeof(cfg_tp5));
    sleep_ms(50);
}

} // namespace drivers