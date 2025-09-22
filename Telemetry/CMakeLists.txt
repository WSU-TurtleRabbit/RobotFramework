add_library(Telemetry Telemetry.cpp)

target_include_directories(Telemetry
    INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
