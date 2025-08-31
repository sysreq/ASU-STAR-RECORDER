#pragma once

// Project Omni-Header
#include "config/all_headers.h"

#include "drivers/sdcard/sdcard.h"
#include "led.h"

namespace logging {

class SessionManager {
public:
    // Enum for file types - easy to extend
    enum FileType {
        FLIGHT = 0,
        GPS,
        PITOT,
        FILE_COUNT  // Must be last
    };

private:
    // File configuration structure
    struct FileConfig {
        const char* filename;
        const char* header;
        bool enabled;
    };
    
    static constexpr FileConfig file_configs[FILE_COUNT] = {
        {"flight.txt", "time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,altitude,pressure,temperature\n", true},
        {"gps.txt", "time_ms,unix_time,latitude,longitude,altitude_mm,vel_north_mm_s,vel_east_mm_s,vel_down_mm_s,heading,h_accuracy,v_accuracy,speed_accuracy,heading_accuracy,valid\n", true},
        {"pitot.txt", "time_ms,airspeed_ms,airspeed_mph,pressure_psi\n", true},
    };
    
    // Pin configuration
    const uint toggle_pin;
    const uint button_pin;
    
    // State tracking
    bool toggle_state = false;
    bool last_toggle_state = false;
    bool button_state = true;
    bool last_button_state = true;
    bool logging_active = false;
    int current_folder_num = -1;
    
    // Double-press detection
    uint32_t last_button_press_time = 0;
    int button_press_count = 0;
    static constexpr uint32_t DOUBLE_PRESS_TIMEOUT_MS = 500;
    bool shutdown_requested = false;
    
    // File handles - now using array
    drivers::SDFile session_files[FILE_COUNT];
    drivers::SDCard& sd_card;
    drivers::SDFile* debug_file;
    
    // Helper to close all files
    void closeAllFiles() {
        for (int i = 0; i < FILE_COUNT; i++) {
            if (session_files[i].isOpen()) {
                session_files[i].sync();
                session_files[i].close();
            }
        }
    }
    
    // Helper to sync all files
    void syncAllFiles() {
        for (int i = 0; i < FILE_COUNT; i++) {
            if (session_files[i].isOpen()) {
                session_files[i].sync();
            }
        }
    }
    
    bool createNewSession() {
        // Close any existing files
        closeAllFiles();
        
        // Find highest numbered folder and increment
        int highest = sd_card.findHighestNumberedFolder();
        current_folder_num = highest + 1;
        
        // Create new folder
        char folder_path[32];
        snprintf(folder_path, sizeof(folder_path), "%d", current_folder_num);
        
        if (!sd_card.createFolder(folder_path)) {
            if (debug_file) {
                debug_file->write("[SESSION][XX] Failed to create folder %s\n", folder_path);
            }
            return false;
        }
        
        // Create all configured files
        bool all_success = true;
        for (int i = 0; i < FILE_COUNT; i++) {
            if (!file_configs[i].enabled) {
                continue;
            }
            
            // Build file path
            char file_path[64];
            snprintf(file_path, sizeof(file_path), "%d/%s", current_folder_num, file_configs[i].filename);
            
            // Open file
            if (!session_files[i].open(file_path, false)) {
                if (debug_file) {
                    debug_file->write("[SESSION][XX] Failed to create %s in folder %d\n", 
                                    file_configs[i].filename, current_folder_num);
                }
                all_success = false;
                continue;
            }
            
            // Write header if provided
            if (file_configs[i].header && strlen(file_configs[i].header) > 0) {
                session_files[i].write(file_configs[i].header);
            }
        }
        
        if (!all_success) {
            closeAllFiles();  // Clean up on failure
            return false;
        }
        
        if (debug_file) {
            debug_file->write("[SESSION][OK] Started session %d\n", current_folder_num);
        }
        
        return true;
    }
    
    void stopLogging() {
        closeAllFiles();
        logging_active = false;
        led_off();
        if (debug_file) {
            debug_file->write("[SESSION][--] Stopped logging\n");
        }
    }
    
public:
    SessionManager(uint toggle, uint button, drivers::SDCard& sd, drivers::SDFile* debug = nullptr) 
        : toggle_pin(toggle), button_pin(button), sd_card(sd), debug_file(debug) {
        
        // Initialize GPIO inputs with pull-ups (switches are active low)
        gpio_init(toggle_pin);
        gpio_set_dir(toggle_pin, GPIO_IN);
        gpio_pull_up(toggle_pin);
        
        gpio_init(button_pin);
        gpio_set_dir(button_pin, GPIO_IN);
        gpio_pull_up(button_pin);
        
        // Initialize LED
        led_init();
        
        // Check initial toggle state at startup
        toggle_state = !gpio_get(toggle_pin);
        last_toggle_state = toggle_state;
        
        if (toggle_state) {
            if (debug_file) {
                debug_file->write("[STARTUP] Toggle is ON - starting logging\n");
            }
            led_on();
            if (createNewSession()) {
                logging_active = true;
            }
        } else {
            if (debug_file) {
                debug_file->write("[STARTUP] Toggle is OFF - waiting to start\n");
            }
            led_off();
        }
    }
    
    ~SessionManager() {
        stopLogging();
    }
    
    void update() {
        // Read current switch states (active low)
        toggle_state = !gpio_get(toggle_pin);
        button_state = gpio_get(button_pin);
        
        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        
        // Button press detection
        if (!button_state && last_button_state) {
            if (logging_active) {
                // During logging - new session
                if (debug_file) {
                    debug_file->write("[BUTTON] Creating new session\n");
                }
                syncAllFiles();  // Sync before creating new session
                createNewSession();
            } else {
                // Not logging - check for double press
                if (current_time - last_button_press_time < DOUBLE_PRESS_TIMEOUT_MS) {
                    button_press_count++;
                    if (button_press_count >= 2) {
                        if (debug_file) {
                            debug_file->write("[SHUTDOWN] Double-press detected - requesting shutdown\n");
                            debug_file->sync();
                        }
                        shutdown_requested = true;
                    }
                } else {
                    button_press_count = 1;
                }
                last_button_press_time = current_time;
            }
        }
        last_button_state = button_state;
        
        // Reset press count if timeout exceeded
        if (current_time - last_button_press_time > DOUBLE_PRESS_TIMEOUT_MS) {
            button_press_count = 0;
        }
        
        // Toggle state change
        if (toggle_state != last_toggle_state) {
            if (toggle_state) {
                // ON - start logging
                if (debug_file) {
                    debug_file->write("[TOGGLE] ON - starting logging\n");
                }
                led_on();
                if (createNewSession()) {
                    logging_active = true;
                }
            } else {
                // OFF - stop logging  
                if (debug_file) {
                    debug_file->write("[TOGGLE] OFF - stopping logging\n");
                }
                stopLogging();
            }
            last_toggle_state = toggle_state;
        }
    }
    
    bool isLogging() const { return logging_active; }
    
    bool isShutdownRequested() const { return shutdown_requested; }
    
    // Generic file getter by type
    drivers::SDFile* getFile(FileType type) { 
        if (type >= FILE_COUNT) return nullptr;
        return session_files[type].isOpen() ? &session_files[type] : nullptr;
    }
    
    // Convenience getters for backward compatibility
    drivers::SDFile* getFlightFile() { return getFile(FLIGHT); }
    drivers::SDFile* getGPSFile() { return getFile(GPS); }
    
    int getCurrentSession() const { return current_folder_num; }
};

} // namespace logging