/**
 * @file EEPROM.h
 * @brief EEPROM interface for read, write, and erase operations.
 * This module provides functions to interact with an EEPROM device,
 * including initialization, reading, writing, and erasing data.
 * 
 * 
 * "C99" standard is used for this project.
 * 
 * @version 1.0
 * 
 * @todo Add DMA and interrupt support for non-blocking operations.
 */

#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// 16K bytes of EEPROM
#define EEPROM_TOTAL_SIZE_BYTES               16384
#define EEPROM_PAGE_SIZE_BYTES                16

static const  uint16_t PROTECTED_EEPROM_START_ADDRESS = 0x0000;
static const  uint16_t PROTECTED_EEPROM_END_ADDRESS   = 0x00FF;

// MEMORY MAP
static const  uint16_t EEPROM_ADDRESS_CONTACTOR_CYCLE_COUNT  = 0x0F00;
static const  uint16_t EEPROM_ADDRESS_CONTACTOR_ACCURUNTIME  = 0x0F10;
static const  uint16_t EEPROM_ADDRESS_DEVICE_CONFIG          = 0x0F20;
static const  uint16_t EEPROM_ADDRESS_SYSTEM_LOGS            = 0x0F30;

// typedef struct __attribute__((__packed__)) {
//     uint8_t data[EEPROM_PAGE_SIZE_BYTES];
// } EEPROM_Page_t;

// typedef struct __attribute__((__packed__)) {
//     uint8_t device_id[8];
//     uint32_t firmware_version;
//     uint32_t calibration_date; 
// } EEPROM_DeviceConfig_t;

typedef struct {
  uint16_t segment_start_index;
  union {
    uint16_t is_protected  : 1;  // 1 bit to indicate if the segment is protected
    uint16_t size_in_bytes : 15; // Size of the segment in bytes (max 32767 bytes)
    // ex. if val is greater than 0x7FFF, then the segment is protected
  } data;
} EEPROM_MemorySegment_t;

/**
  * @brief Initializes the EEPROM hardware.
  *
  * @return true if initialization is successful, false otherwise.
  */
bool EEPROM_Init(void);

/**
  * @brief Reads data from EEPROM.
  *
  * @param address The starting address to read from.
  * @param buffer Pointer to the buffer to store read data.
  * @param length Number of bytes to read.
  * @return true if read operation is successful, false otherwise.
  */
bool EEPROM_Read(uint16_t address, uint8_t* buffer, size_t length);

/**
  * @brief Writes data to EEPROM.
  *
  * @param address The starting address to write to.
  * @param data Pointer to the data to write.
  * @param length Number of bytes to write.
  * @return true if write operation is successful, false otherwise.
  */
bool EEPROM_Write(uint16_t address, const uint8_t* data, size_t length);

/**
  * @brief Erases a section of the EEPROM.
  *
  * @param address The starting address to erase.
  * @param length Number of bytes to erase.
  * @return true if erase operation is successful, false otherwise.
  */
bool EEPROM_Erase(uint16_t address, size_t length);

/**
 * @brief Checks if the EEPROM is ready for the next operation.
 * 
 * @return true if EEPROM is ready, false if it is busy.
 */
bool EEPROM_isReady(void);

#endif // EEPROM_H