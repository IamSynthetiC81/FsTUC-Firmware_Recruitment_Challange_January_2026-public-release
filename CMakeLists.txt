#cmake_minimum_required(VERSION 3.20)
#
#project(FsTUC_Firmware_Recruitment
#VERSION 2.6.0
#LANGUAGES C
#)
#
## ============================================================================
## Cross-Platform Compiler Settings
## ============================================================================
#set(CMAKE_C_STANDARD 11)
#set(CMAKE_C_STANDARD_REQUIRED ON)
#
#if(MSVC)
## Windows-specific flags (Visual Studio)
#add_compile_options(/W4 /WX)
#else()
## GCC/Clang flags (Linux/Mac/WSL)
#add_compile_options(-Wall -Wextra -Werror -g -DUNIT_TESTING)
#endif()
#
## ============================================================================
## Fetch Unity test framework
## ============================================================================
#include(FetchContent)
#FetchContent_Declare(
#unity
#GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
#GIT_TAG        v2.5.2
#)
#FetchContent_MakeAvailable(unity)
#
#add_library(unity_lib STATIC ${unity_SOURCE_DIR}/src/unity.c)
#target_include_directories(unity_lib PUBLIC ${unity_SOURCE_DIR}/src)
#
## ============================================================================
## Imported Hidden Library (The "Blob")
## ============================================================================
## This points to the library YOU pre-compiled.
#add_library(EmbeddedHardware STATIC IMPORTED)
#set_target_properties(EmbeddedHardware PROPERTIES
#IMPORTED_LOCATION "${CMAKE_SOURCE_DIR}/lib/libEmbeddedHardware${CMAKE_STATIC_LIBRARY_SUFFIX}"
#INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#)
#
## Paths for participant files
#set(PARTICIPANT_SRC_DIR ${CMAKE_SOURCE_DIR}/src)
#set(MOCK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/tests/mocks)
#
## ============================================================================
## Helper macro: define a test executable
## ============================================================================
#macro(add_test_target TARGET_NAME)
#set(TEST_SOURCES ${ARGN})
#add_executable(${TARGET_NAME} ${TEST_SOURCES})
#
#target_include_directories(${TARGET_NAME} PRIVATE
#${CMAKE_SOURCE_DIR}/include
#${MOCK_INCLUDE_DIR}
#${unity_SOURCE_DIR}/src
#)
#
## Link against Unity and your hidden binary
#target_link_libraries(${TARGET_NAME} PRIVATE
#unity_lib
#EmbeddedHardware
#)
#
## Link math library only on Unix systems (Linux/Mac/WSL)
#if(UNIX)
#target_link_libraries(${TARGET_NAME} PRIVATE m)
#endif()
#
#add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
#endmacro()
#
## ============================================================================
## Test Targets (Participant builds these)
## ============================================================================
#
## Participants implement contactor.c
#add_test_target(test_contactor
#${PARTICIPANT_SRC_DIR}/contactor.c
#tests/mocks/HAL_io_mock.c
#tests/Contactor_Runner.c
#)
#
## Participants implement BatteryPack.c and FSM.c
#add_test_target(test_battery_pack
#${PARTICIPANT_SRC_DIR}/BatteryPack.c
#${PARTICIPANT_SRC_DIR}/FSM.c
#tests/mocks/HAL_io_mock.c
#tests/Test_Utils.c
#tests/BatteryPack_Test_Runner.c
#)
#
## ... (Add other targets test_eeprom, test_adc following the same logic)
#
## ============================================================================
## Custom Utility Targets
## ============================================================================
#find_package(Python3 COMPONENTS Interpreter)
#
#add_custom_target(run_all_tests
#COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
#COMMENT "Running all test suites via CTest"
#)
cmake_minimum_required(VERSION 3.20)

project(FsTUC_Firmware_Recruitment
        VERSION 2.6.0
        LANGUAGES C
)

enable_testing()

# ============================================================================
# Global settings
# ============================================================================
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

option(USE_PREBUILT_EMBEDDED "Link tests against prebuilt EmbeddedHardware static library" OFF)
set(PREBUILT_EMBEDDED_DIR "${CMAKE_SOURCE_DIR}/prebuilt/${CMAKE_SYSTEM_NAME}" CACHE PATH "Directory containing prebuilt EmbeddedHardware static library")

# ============================================================================
# Fetch Unity test framework
# ============================================================================
include(FetchContent)

FetchContent_Declare(
        unity
        GIT_REPOSITORY https://github.com/ThrowTheSwitch/Unity.git
        GIT_TAG        v2.5.2
)
FetchContent_MakeAvailable(unity)

# ============================================================================
# Unity static library (shared by all test targets)
# ============================================================================
add_library(unity_lib STATIC
        ${unity_SOURCE_DIR}/src/unity.c
)
target_include_directories(unity_lib PUBLIC
        ${unity_SOURCE_DIR}/src
)

# ============================================================================
# Common include paths
# ============================================================================
set(LIB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/lib/include)
set(MOCK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/tests/mocks)

# ============================================================================
# Common compiler flags
# ============================================================================
set(COMMON_FLAGS
                -DUNIT_TESTING
)

if(MSVC)
        list(APPEND COMMON_FLAGS /W4)
else()
        list(APPEND COMMON_FLAGS -Wall -Wextra -g)
endif()

# ============================================================================
# Embedded Hardware library (production code, no UNIT_TESTING)
# ============================================================================
if(USE_PREBUILT_EMBEDDED)
        set(EMBEDDED_PREBUILT_PATH "${PREBUILT_EMBEDDED_DIR}/${CMAKE_STATIC_LIBRARY_PREFIX}EmbeddedHardware${CMAKE_STATIC_LIBRARY_SUFFIX}")
        if(NOT EXISTS "${EMBEDDED_PREBUILT_PATH}")
                message(FATAL_ERROR "Prebuilt EmbeddedHardware library not found: ${EMBEDDED_PREBUILT_PATH}")
        endif()

    add_library(EmbeddedHardware STATIC IMPORTED GLOBAL)
    set_target_properties(EmbeddedHardware PROPERTIES
                        IMPORTED_LOCATION "${EMBEDDED_PREBUILT_PATH}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIB_INCLUDE_DIR}"
    )
else()
    add_library(EmbeddedHardware STATIC
            lib/src/ADC.c
            lib/src/EEPROM.c
            lib/src/Errno.c
            lib/src/HAL_io.c
            lib/src/CircuitSim.c
            lib/src/Clock.c
    )
    target_include_directories(EmbeddedHardware PUBLIC
            ${LIB_INCLUDE_DIR}
            ${MOCK_INCLUDE_DIR}
    )

    if(MSVC)
        target_compile_options(EmbeddedHardware PRIVATE /W4)
    else()
        target_compile_options(EmbeddedHardware PRIVATE -Wall -Wextra -g)
    endif()
endif()

# ============================================================================
# Helper macro: define a test executable
# ============================================================================
macro(add_test_target TARGET_NAME)
    set(TEST_SOURCES ${ARGN})

    add_executable(${TARGET_NAME} ${TEST_SOURCES})

    target_include_directories(${TARGET_NAME} PRIVATE
            ${LIB_INCLUDE_DIR}
            ${MOCK_INCLUDE_DIR}
            ${unity_SOURCE_DIR}/src
    )

    target_compile_options(${TARGET_NAME} PRIVATE ${COMMON_FLAGS})

        if(MSVC)
                target_link_libraries(${TARGET_NAME} PRIVATE unity_lib EmbeddedHardware)
        else()
                target_link_libraries(${TARGET_NAME} PRIVATE unity_lib EmbeddedHardware m)
        endif()

    add_test(NAME ${TARGET_NAME} COMMAND ${TARGET_NAME})
endmacro()

# ============================================================================
# test_contactor
# ============================================================================
add_test_target(test_contactor
        lib/src/contactor.c
        tests/mocks/HAL_io_mock.c
        tests/Contactor_Runner.c
)

# ============================================================================
# test_eeprom
# ============================================================================
add_test_target(test_eeprom
        tests/mocks/HAL_io_mock.c
        tests/EEPROM_Runner.c
)

# ============================================================================
# test_adc
# ============================================================================
add_test_target(test_adc
        lib/src/BatteryPack.c
        lib/src/FSM.c
        lib/src/contactor.c
        tests/mocks/HAL_io_mock.c
        tests/ADC_Test_Runner.c
)

# ============================================================================
# test_battery_pack
# ============================================================================
add_test_target(test_battery_pack
        lib/src/BatteryPack.c
        lib/src/FSM.c
        lib/src/contactor.c
        tests/mocks/HAL_io_mock.c
        tests/Test_Utils.c
        tests/BatteryPack_Test_Runner.c
)


# ============================================================================
# Convenience targets
# ============================================================================

add_custom_target(build_all
        DEPENDS
        test_contactor
        test_eeprom
        test_adc
        test_battery_pack
        COMMENT "Build all test targets"
)

if(NOT USE_PREBUILT_EMBEDDED)
    add_custom_target(export_embedded_prebuilt
            COMMAND ${CMAKE_COMMAND} -E make_directory ${PREBUILT_EMBEDDED_DIR}
            COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:EmbeddedHardware> ${PREBUILT_EMBEDDED_DIR}/
            DEPENDS EmbeddedHardware
            COMMENT "Export EmbeddedHardware static library to prebuilt folder"
    )
endif()

add_custom_target(run_test_contactor
        COMMAND $<TARGET_FILE:test_contactor>
        DEPENDS test_contactor
        COMMENT "Run test_contactor"
        USES_TERMINAL
)

add_custom_target(run_test_eeprom
        COMMAND $<TARGET_FILE:test_eeprom>
        DEPENDS test_eeprom
        COMMENT "Run test_eeprom"
        USES_TERMINAL
)

add_custom_target(run_test_adc
        COMMAND $<TARGET_FILE:test_adc>
        DEPENDS test_adc
        COMMENT "Run test_adc"
        USES_TERMINAL
)

add_custom_target(run_test_battery_pack
        COMMAND $<TARGET_FILE:test_battery_pack>
        DEPENDS test_battery_pack
        COMMENT "Run test_battery_pack"
        USES_TERMINAL
)

add_custom_target(run_all_tests
        COMMAND $<TARGET_FILE:test_contactor>
        COMMAND $<TARGET_FILE:test_eeprom>
        COMMAND $<TARGET_FILE:test_adc>
        COMMAND $<TARGET_FILE:test_battery_pack>
        DEPENDS build_all
        COMMENT "Run all test suites"
        USES_TERMINAL
)

add_custom_target(generate_mocks
        COMMAND ${CMAKE_COMMAND} -E env bash ${CMAKE_SOURCE_DIR}/scripts/generate_mocks.sh
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generate CMock mocks"
        USES_TERMINAL
)

find_package(Python3 COMPONENTS Interpreter)
add_custom_target(generate_report
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/generate_report.py
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generate HTML test report"
        USES_TERMINAL
)

add_custom_target(deep_clean
        COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}
        COMMENT "Remove build directory"
)
