// Project Omni-Header
#include "config/all_headers.h"


#include "drivers/sensors/i2c_bus.h"
#include "drivers/gps/gps_driver.h"
#include "drivers/sensors/icm20948_driver.h"
#include "drivers/sensors/bmp581_driver.h"
#include "drivers/sensors/pitot_tube.h"
#include "drivers/sdcard/sdcard.h"


#include "config/config.h"

#include "led.h"
#include "session_manager.h"

void StartProcess() {
    stdio_init_all();

    if(false) {
        while(!stdio_usb_connected()) {
            sleep_ms(10);
            continue;
        }
    }
    
    sleep_ms(250);
    printf("Welcome.\n");
    sleep_ms(250);
}

int EndProcess() {
    sleep_ms(500);
    printf("\nGoodbye.\n");
    sleep_ms(500);
    reset_block_num(RESET_USBCTRL);

    int i = 0;
    while(stdio_usb_connected() && i < 100) {
        sleep_ms(10);
        i++;
        continue;
    }
    reset_usb_boot(0, 0);

    return 0;
}

int Error() {
    led_init();
    sleep_ms(50);
    led_pulse_init();
    sleep_ms(50);
    while(true) { led_pulse(); }
    return -1;
}

int main()
{
    using namespace config;
    using namespace config::system;
    using namespace drivers;
    using FileType = logging::SessionManager::FileType;

    StartProcess();

    
    // Initialize SD Card driver - exactly like GPS pattern
    auto& sd = SDCard::instance();
    
    if (!sd.mount()) {
        printf("[SDCARD][XX] Failed to mount SD card\n");
        return Error();
    }

    // Create persistent debug log
    SDFile debug;
    if (!debug.open("debug.txt", true)) { // Append mode
        printf("[SDCARD][XX] Failed to create debug log\n");
        return Error();
    }
    debug.write("\n----- STARTED -----\n");

    // Initialize session manager (checks toggle state at startup)
    logging::SessionManager sessions(TOGGLE_PIN, BUTTON_PIN, sd, &debug);

    // Initialize GPS
    drivers::GpsDriver gps;
    gps.init(uart0, gps::RX_PIN, gps::TX_PIN, gps::USE_BINARY_UBX);
    gps.set_led_enabled(true);

    // Initialize sensors
    I2CBus i2c_bus;
    if(i2c_bus.init(i2c0, i2c::bus0::SDA, i2c::bus0::SCL, i2c::bus0::DATA_RATE))
        debug.write("[I2CBUS][OK] I2CBus initialized successfully\n");

    ICM20948 icm20948;
    if(icm20948.init(&i2c_bus))
        debug.write("[ICU948][OK] ICM20948 initialized successfully\n");

    BMP581 bmp581;
    if(bmp581.init(&i2c_bus))
        debug.write("[BMP581][OK] BMP581 initialized successfully\n");

    PitotTube pitot_tube;
    if(pitot_tube.init(&i2c_bus, 1.0f))
        debug.write("[PITOTT][OK] PitotTube initialized successfully\n");
    sleep_ms(50);
    if(pitot_tube.calibrate_zero(50))
        debug.write("[PITOTT][OK] PitotTube calibrated successfully\n");

    uint32_t start = to_ms_since_boot(time_us_64());
    uint32_t now = start;
    uint32_t last_raw = 0;
    uint32_t last_pitot = 0;

    printf("==== STARTING LOOP ====\n");
    
    while (true) {
        now = to_ms_since_boot(time_us_64());
        
        sessions.update();
        if (sessions.isShutdownRequested()) {
            break;
        }
        
        // Log data if session is active
        if (sessions.isLogging()) {
            
            if (now - last_raw > utils::hz_to_ms(sensors::RAW_DATA_HZ)) {
                auto* file = sessions.getFile(FileType::FLIGHT);          
                if (file && file->isOpen() && icm20948.update() && bmp581.update()) {
                    auto icm = icm20948.get_data();
                    auto bmp = bmp581.get_data();

                    file->write("%" PRIu32 ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                        now,
                        icm.accel_x, icm.accel_y, icm.accel_z,
                        icm.gyro_x, icm.gyro_y, icm.gyro_z,
                        bmp.altitude, bmp.pressure, bmp.temperature);

                    last_raw = now;
                }
            }

            if(gps.update()) {
                auto data = gps.get_data();
                auto* file = sessions.getFile(FileType::GPS);
                if (file && file->isOpen()) {
                    file->write("%" PRIu32 ",%" PRIu32 ",%.6f,%.6f,%d,%d,%d,%d,%d,%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%" PRIu32 ",%s\n",
                        now, data.unix_time, data.lat, data.lon, data.hMSL,
                        data.velN, data.velE, data.velD, data.heading, data.hAcc, data.vAcc, 
                        data.sAcc, data.headingAcc, data.valid ? "OK" : "NO");
                }
                gps.clear();
            }

            if (now - last_pitot > utils::hz_to_ms(sensors::PITOT_RATE_HZ)) {
                auto* file = sessions.getFile(FileType::PITOT);          
                if (file && file->isOpen() && pitot_tube.update()) {
                    auto pitot = pitot_tube.get_data();
                    
                    if (pitot.valid) {
                        file->write("%" PRIu32 ",%.2f,%.2f,%.2f,%.2f\n",
                            now,
                            pitot.airspeed_ms,
                            pitot.airspeed_mph,
                            pitot.pressure_psi);
                    }
                    
                    last_pitot = now;
                }
            }

        }

        sleep_ms(1);
    }

    debug.sync();
    debug.close();
    sd.shutdown();

    sleep_ms(100);
    return EndProcess();
}
