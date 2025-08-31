#pragma once

// Project Omni-Header
#include "config/all_headers.h"

extern "C" {
    #include "ff.h"
    #include "f_util.h"
    #include "sd_card.h"
}

namespace drivers {

// Forward declaration
class SDFile;

// ============================================
// Simplified SD Card Driver (Singleton)
// ============================================
class SDCard {
private:
    // SD Card structures - statically allocated
    spi_t spi_config;
    sd_spi_if_t spi_if;
    sd_card_t sd_card;
    FATFS fs;
    
    bool initialized = false;
    bool mounted = false;
    
    // Track open files
    static constexpr size_t MAX_FILES = 8;
    uint8_t open_files = 0;
    
    // Private constructor for singleton
    SDCard();
    
public:
    // Get singleton instance
    static SDCard& instance();
    
    // Delete copy/move operations
    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;
    SDCard(SDCard&&) = delete;
    SDCard& operator=(SDCard&&) = delete;
    
    // Core operations - pins come from config
    bool init();  // Uses CONFIG_SD_* constants
    bool mount();
    void shutdown();
    
    // File system queries
    bool hasFile(const char* path);
    bool hasFolder(const char* path);
    
    // Folder operations (minimal additions)
    bool createFolder(const char* path);
    int findHighestNumberedFolder(const char* prefix = "");
    
    // Status
    bool isMounted() const { return mounted; }
    bool isInitialized() const { return initialized; }
    sd_card_t* getCardPtr() { return initialized ? &sd_card : nullptr; }
    
    // Internal use by SDFile
    bool registerFile();
    void unregisterFile();
    
    friend class SDFile;
};

// ============================================
// Simplified File Class
// ============================================
class SDFile {
private:
    FIL fil;
    bool is_open = false;
    
    // Single write buffer
    static constexpr size_t BUFFER_SIZE = 512;
    uint8_t buffer[BUFFER_SIZE];
    size_t buffer_pos = 0;
    
    bool flushBuffer();
    
public:
    SDFile();
    ~SDFile();
    
    // No copy/move
    SDFile(const SDFile&) = delete;
    SDFile& operator=(const SDFile&) = delete;
    SDFile(SDFile&&) = delete;
    SDFile& operator=(SDFile&&) = delete;
    
    // File operations
    bool open(const char* path, bool append = false);
    bool write(const char* format, ...);
    bool sync();
    bool close();
    
    // Status
    bool isOpen() const { return is_open; }
};

} // namespace drivers

// C interface functions for FatFS
extern "C" {
    size_t sd_get_num();
    sd_card_t* sd_get_by_num(size_t num);
}