cmake_minimum_required(VERSION 2.6)
project (rootproject)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
 
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
 
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
 
include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/util_pkt_logger/inc")
include_directories("${PROJECT_SOURCE_DIR}/libloragw/inc")
 
#put libloragw.a where packet_forwarder expects it:
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libloragw)

add_subdirectory(libloragw)

list(APPEND util_pkt_logger_srcs
    ${PROJECT_SOURCE_DIR}/util_pkt_logger/src/util_pkt_logger.c
    ${PROJECT_SOURCE_DIR}/util_pkt_logger/src/parson.c
)

add_executable(util_pkt_logger ${util_pkt_logger_srcs})
add_dependencies(util_pkt_logger loragw)
find_library(M_LIB m)
find_library(P_LIB pthread)
#target_link_libraries(util_pkt_logger ${CMAKE_BINARY_DIR}/libloragw.a ${M_LIB})
target_link_libraries(util_pkt_logger
    ${PROJECT_SOURCE_DIR}/libloragw/libloragw.a
    ${M_LIB}
    ${P_LIB}
)

