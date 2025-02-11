cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bhi360_demo)

# Add BHY2 sensor API source files
file(GLOB BHY2_SOURCES 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/BHY2-Sensor-API/*.c"
)

target_sources(app PRIVATE
    src/main.c
    src/common.c
    ${BHY2_SOURCES}
)

# Include directories - simplified paths
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/BHY2-Sensor-API
)

# Add compile definitions if needed
target_compile_definitions(app PRIVATE
    BHY2_CFG_DELEGATE_FIFO_PARSE_CB_INFO_MGMT=0
)