#include "EEPROM.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define EEPROM_WRITE_CYCLE_MS 5 // Simulated write cycle time per page in milliseconds

static uint8_t eeprom_memory[EEPROM_TOTAL_SIZE_BYTES]; // Simulated EEPROM memory of 16KB

static clock_t busy_until = 0;

static bool is_busy(void){
    return clock() < busy_until;
}

bool EEPROM_Init(void) {
    const char* restricted_str = "RESTRICTED";  
    for (uint16_t i = 0; i <= PROTECTED_EEPROM_END_ADDRESS ; i++){
        size_t restricted_len = strlen(restricted_str);
        eeprom_memory[i] = restricted_str[i % restricted_len];
    } 
    
    (void)memset(&eeprom_memory[PROTECTED_EEPROM_END_ADDRESS + 1], 0xFF, EEPROM_TOTAL_SIZE_BYTES - (PROTECTED_EEPROM_END_ADDRESS + 1));
    return true;
}

bool EEPROM_Read(uint16_t address, uint8_t* buffer, size_t length) {
    if (address + length > EEPROM_TOTAL_SIZE_BYTES || buffer == NULL) {
        return false; // Out of bounds or null buffer
    }
    uint8_t* retval = memcpy(buffer, &eeprom_memory[address], length);

    return retval == buffer;
}

bool EEPROM_Write(uint16_t address, const uint8_t* data, size_t length) {
    if (is_busy()) return false;
    
    if (data == NULL || address >= EEPROM_TOTAL_SIZE_BYTES) return false;
    if (address + length <= PROTECTED_EEPROM_END_ADDRESS) return false; // Entire write is in protected region
    size_t _bottom_overlap = 0, _top_overlap = 0;

    _bottom_overlap = (address <= PROTECTED_EEPROM_END_ADDRESS) ? (size_t)(PROTECTED_EEPROM_END_ADDRESS - address + 1) : 0;
    _top_overlap = ((address + length -1) > EEPROM_TOTAL_SIZE_BYTES) ? (size_t)(address + length - EEPROM_TOTAL_SIZE_BYTES) : 0;

    // If the write overlaps with protected region, clip it
    _bottom_overlap = (_bottom_overlap > length) ? length : _bottom_overlap;
    _top_overlap = (_top_overlap > (length - _bottom_overlap)) ? (length - _bottom_overlap) : _top_overlap;

    length  -= (_bottom_overlap + _top_overlap);
    address += _bottom_overlap;
    data    += _bottom_overlap;

    // If nothing left to write after removing protected regions, return success (best-effort)
    if (length == 0) {
        return true;
    }

    int num_pages = 1 + ((address + length -1) / EEPROM_PAGE_SIZE_BYTES) - (address / EEPROM_PAGE_SIZE_BYTES);

    memmove(&eeprom_memory[address], data, length);

    // busy_until = clock() + delay_in_clocks
    // EEPROM_WRITE_CYCLE_MS = 5 ms per page
    // Convert to clock ticks: (milliseconds * CLOCKS_PER_SEC) / 1000
    busy_until = clock() + (num_pages * EEPROM_WRITE_CYCLE_MS * CLOCKS_PER_SEC) / 1000;

    return true;
}

bool EEPROM_Erase(uint16_t address, size_t length) {
    if (address + length > EEPROM_TOTAL_SIZE_BYTES) {
        return false; // Out of bounds
    }

    size_t _bottom_overlap = 0, _top_overlap = 0;
    _bottom_overlap = (address < PROTECTED_EEPROM_END_ADDRESS) ? (size_t)(PROTECTED_EEPROM_END_ADDRESS - address + 1) : 0;
    _top_overlap = ((address + length ) > EEPROM_TOTAL_SIZE_BYTES) ? (size_t)(address + length - EEPROM_TOTAL_SIZE_BYTES) : 0;
    length  -= _bottom_overlap + _top_overlap;
    address += _bottom_overlap;

    (void*)memset(&eeprom_memory[address], 0xFF, length);

    return true;
}   

bool EEPROM_isReady(void) {
    return clock() >= busy_until;
}