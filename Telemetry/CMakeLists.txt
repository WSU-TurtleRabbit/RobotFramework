add_library(Telemetry Telemetry.cpp)

target_include_directories(Telemetry
    INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Telemetry PUBLIC
    moteus 
    pi3hat
)