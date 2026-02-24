#include "unity.h"
#include "EEPROM.h"
#include <stdio.h>
#include <string.h>


void setUp(void) {
    // This runs BEFORE every RUN_TEST
    // 1. Initialize and wipe non-protected EEPROM
    EEPROM_Init();
    // Erase only the non-protected region
    uint16_t start = PROTECTED_EEPROM_END_ADDRESS + 1;
    EEPROM_Erase(start, EEPROM_TOTAL_SIZE_BYTES - start);
    // Wait for EEPROM to finish erasing
    while(!EEPROM_isReady());
}

void tearDown(void) {
    // This runs AFTER every RUN_TEST
}

void test_EEPROM_WriteProtection(void) {
    uint8_t secret_data = 0xAA;
    uint8_t read_val = 0;
    
    // Attempt to write to protected address 0
    // We expect the function to return false (The write is rejected)
    // but the memory should NOT change.
    TEST_ASSERT_FALSE_MESSAGE(EEPROM_Write(0, &secret_data, 1), "Write command not rejected"); 
    
    EEPROM_Read(0, &read_val, 1);
    
    // Using HEX8 makes failure logs much easier to read (e.g., "Expected 0xFF, Was 0xAA")
    TEST_ASSERT_NOT_EQUAL_HEX8_MESSAGE(0xAA, read_val, "Protected memory was overwritten!");
}

void test_EEPROM_ReadWrite(void) {
    uint8_t data_in = 0x55;
    uint8_t data_out = 0;
    uint16_t addr = PROTECTED_EEPROM_END_ADDRESS + 1;

    TEST_ASSERT_TRUE(EEPROM_Write(addr, &data_in, sizeof(data_in)));
    
    while(!EEPROM_isReady()); 

    TEST_ASSERT_TRUE(EEPROM_Read(addr, &data_out, sizeof(data_out)));
    TEST_ASSERT_EQUAL_HEX8(data_in, data_out);
}

void test_EEPROM_Erase(void) {
    uint8_t data_in = 0x77;
    uint8_t data_out = 0;
    uint16_t addr = PROTECTED_EEPROM_END_ADDRESS + 10;
    size_t length = 5;

    // Write known data
    for (size_t i = 0; i < length; i++) {
        while(!EEPROM_isReady());
        TEST_ASSERT_TRUE_MESSAGE(EEPROM_Write(addr + i, &data_in, 1), "Setup write failed");
    }

    while(!EEPROM_isReady()); 

    // Erase the section
    TEST_ASSERT_TRUE(EEPROM_Erase(addr, length));

    while(!EEPROM_isReady()); 

    // Verify erased data
    for (size_t i = 0; i < length; i++) {
        TEST_ASSERT_TRUE(EEPROM_Read(addr + i, &data_out, 1));
        TEST_ASSERT_EQUAL_HEX8_MESSAGE(0xFF, data_out, "Memory was not erased to 0xFF");
    }
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_EEPROM_WriteProtection);
    RUN_TEST(test_EEPROM_ReadWrite);
    RUN_TEST(test_EEPROM_Erase);

    return UNITY_END();
}