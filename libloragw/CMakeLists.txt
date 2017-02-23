
set (DEBUG_AUX      0)
set (DEBUG_SPI      0)
set (DEBUG_REG      0)
set (DEBUG_HAL      0)
set (DEBUG_GPS      1)
set (DEBUG_GPIO     0)
set (DEBUG_LBT      0)

cmake_minimum_required(VERSION 2.6)
project (loragw)

file (STRINGS "${CMAKE_SOURCE_DIR}/VERSION" LIBLORAGW_VERSION)
 
set (PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/inc)
set (PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
 
if (SHM)
set (PROJECT1_SRCS 
    ${PROJECT_SOURCE_DIR}/loragw_hal.shm.c
    ${PROJECT_SOURCE_DIR}/loragw_gps.shm.c
    ${PROJECT_SOURCE_DIR}/loragw_aux.c
)
else (SHM)
set (PROJECT1_SRCS 
    ${PROJECT_SOURCE_DIR}/loragw_hal.c
    ${PROJECT_SOURCE_DIR}/loragw_reg.c
    ${PROJECT_SOURCE_DIR}/loragw_lbt.c
    ${PROJECT_SOURCE_DIR}/loragw_aux.c
    ${PROJECT_SOURCE_DIR}/loragw_fpga.c
    ${PROJECT_SOURCE_DIR}/loragw_spi.native.c
    ${PROJECT_SOURCE_DIR}/loragw_radio.c
)
endif (SHM)

if (UBX)
    add_definitions(-DENABLE_HAL_UBX)
endif (UBX)
 
include_directories ("${PROJECT_BINARY_DIR}")
include_directories ("${PROJECT_INCLUDE_DIR}")
 
add_library (${PROJECT_NAME} STATIC ${PROJECT1_SRCS})
set_property (TARGET ${PROJECT_NAME} PROPERTY C_STANDARD 99)

set (CONF_HDR ${PROJECT_INCLUDE_DIR}/config.h)
file (WRITE ${CONF_HDR} "#ifndef _LORAGW_CONFIGURATION_H\n")
file (APPEND ${CONF_HDR} "#define _LORAGW_CONFIGURATION_H\n")
file (APPEND ${CONF_HDR} "#define LIBLORAGW_VERSION \"${LIBLORAGW_VERSION}\"\n")
file (APPEND ${CONF_HDR} "#define DEBUG_AUX ${DEBUG_AUX}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_SPI ${DEBUG_SPI}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_REG ${DEBUG_REG}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_HAL ${DEBUG_HAL}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_GPS ${DEBUG_GPS}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_GPIO ${DEBUG_GPIO}\n")
file (APPEND ${CONF_HDR} "#define DEBUG_LBT ${DEBUG_LBT}\n")
file (APPEND ${CONF_HDR} "#endif\n")

