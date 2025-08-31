#include "sdcard.h"
#include "config/config.h"

// C interface for FatFS library
extern "C" {
    size_t sd_get_num() { 
        auto& sd = drivers::SDCard::instance();
        return sd.isInitialized() ? 1 : 0;
    }
    
    sd_card_t* sd_get_by_num(size_t num) {
        if (num != 0) return nullptr;
        auto& sd = drivers::SDCard::instance();
        return sd.getCardPtr();
    }
}

namespace drivers {

// ============================================
// SDCard Implementation
// ============================================

SDCard::SDCard() {
    memset(&spi_config, 0, sizeof(spi_config));
    memset(&spi_if, 0, sizeof(spi_if));
    memset(&sd_card, 0, sizeof(sd_card));
    memset(&fs, 0, sizeof(fs));
}

SDCard& SDCard::instance() {
    static SDCard instance;
    return instance;
}

bool SDCard::init() {
    using namespace config::sdcard;
    if (initialized) return true;
    
    // Configure SPI using static config constants
    spi_config.hw_inst = SPI_BUS;
    spi_config.miso_gpio = MISO;
    spi_config.mosi_gpio = MOSI;
    spi_config.sck_gpio = SCK;
    spi_config.baud_rate = FREQ_HZ;
    
    // Configure SD interface
    spi_if.spi = &spi_config;
    spi_if.ss_gpio = CS;
    
    // Configure SD card
    sd_card.type = SD_IF_SPI;
    sd_card.spi_if_p = &spi_if;
    
    initialized = true;
    return true;
}

bool SDCard::mount() {
    if (!initialized && !init()) {
        return false;
    }
    
    if (mounted) return true;
    
    FRESULT result = f_mount(&fs, "", 1);
    mounted = (result == FR_OK);
    return mounted;
}

void SDCard::shutdown() {
    if (open_files > 0) {
        // Force close all files (emergency shutdown)
        open_files = 0;
    }
    
    if (mounted) {
        f_unmount("");
        mounted = false;
    }
    
    initialized = false;
}

bool SDCard::hasFile(const char* path) {
    if (!mounted) return false;
    
    FILINFO fno;
    FRESULT res = f_stat(path, &fno);
    
    if (res != FR_OK) return false;
    
    // Check it's a file (not directory)
    return !(fno.fattrib & AM_DIR);
}

bool SDCard::hasFolder(const char* path) {
    if (!mounted) return false;
    
    FILINFO fno;
    FRESULT res = f_stat(path, &fno);
    
    if (res != FR_OK) return false;
    
    // Check it's a directory
    return (fno.fattrib & AM_DIR);
}

bool SDCard::createFolder(const char* path) {
    if (!mounted) return false;
    
    FRESULT res = f_mkdir(path);
    // FR_EXIST is okay - folder already exists
    return (res == FR_OK || res == FR_EXIST);
}

int SDCard::findHighestNumberedFolder(const char* prefix) {
    if (!mounted) return -1;
    
    DIR dir;
    FILINFO fno;
    int highest = -1;
    
    // Open root directory or specified prefix
    FRESULT res = f_opendir(&dir, prefix);
    if (res != FR_OK) return -1;
    
    while (true) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break; // End of dir
        
        // Only check directories
        if (fno.fattrib & AM_DIR) {
            // Try to parse as number
            char* endptr;
            int num = strtol(fno.fname, &endptr, 10);
            
            // Check if entire name was a number
            if (*endptr == '\0' && num >= 0) {
                if (num > highest) {
                    highest = num;
                }
            }
        }
    }
    
    f_closedir(&dir);
    return highest;
}

bool SDCard::registerFile() {
    if (open_files >= MAX_FILES) return false;
    open_files++;
    return true;
}

void SDCard::unregisterFile() {
    if (open_files > 0) open_files--;
}

// ============================================
// SDFile Implementation
// ============================================

SDFile::SDFile() {
    memset(buffer, 0, BUFFER_SIZE);
}

SDFile::~SDFile() {
    if (is_open) {
        close();
    }
}

bool SDFile::flushBuffer() {
    if (buffer_pos == 0) return true;
    
    UINT written;
    FRESULT fr = f_write(&fil, buffer, buffer_pos, &written);
    
    if (fr != FR_OK || written != buffer_pos) {
        return false;
    }
    
    buffer_pos = 0;
    return true;
}

bool SDFile::open(const char* path, bool append) {
    if (is_open) close();
    
    auto& sd = SDCard::instance();
    if (!sd.isMounted() || !sd.registerFile()) {
        return false;
    }
    
    uint8_t mode = FA_WRITE | FA_CREATE_ALWAYS;
    if (append) {
        mode = FA_WRITE | FA_OPEN_APPEND;
    }
    
    FRESULT fr = f_open(&fil, path, mode);
    is_open = (fr == FR_OK);
    
    if (!is_open) {
        sd.unregisterFile();
    }
    
    buffer_pos = 0;
    return is_open;
}

bool SDFile::write(const char* format, ...) {
    if (!is_open) return false;
    
    // Use a temporary buffer for formatting
    char temp[256];
    
    va_list args;
    va_start(args, format);
    int len = vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);
    
    if (len < 0) return false;
    
    // Print the formatted string to console
    printf("%s", temp);
    
    // Handle case where string is larger than temp buffer
    if (len >= sizeof(temp)) {
        // Truncate to fit - could alternatively dynamically allocate
        len = sizeof(temp) - 1;
    }
    
    // Write the formatted string
    const uint8_t* src = reinterpret_cast<const uint8_t*>(temp);
    size_t bytes_to_write = len;
    
    while (bytes_to_write > 0) {
        size_t space = BUFFER_SIZE - buffer_pos;
        size_t to_copy = (bytes_to_write < space) ? bytes_to_write : space;
        
        memcpy(buffer + buffer_pos, src, to_copy);
        buffer_pos += to_copy;
        src += to_copy;
        bytes_to_write -= to_copy;
        
        if (buffer_pos >= BUFFER_SIZE) {
            if (!flushBuffer()) return false;
        }
    }
    
    return true;
}

bool SDFile::sync() {
    if (!is_open) return false;
    
    if (!flushBuffer()) return false;
    
    return (f_sync(&fil) == FR_OK);
}

bool SDFile::close() {
    if (!is_open) return true;
    
    if (!flushBuffer()) return false;
    
    is_open = false;
    FRESULT result = f_close(&fil);
    
    SDCard::instance().unregisterFile();
    
    return (result == FR_OK);
}

} // namespace drivers