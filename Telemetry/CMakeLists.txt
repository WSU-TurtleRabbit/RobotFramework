add_library(Telemetry Telemetry.cpp)

target_include_directories(Telemetry
    INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/mjbots
    ${CMAKE_SOURCE_DIR}/mjbots/moteus
    ${CMAKE_SOURCE_DIR}/mjbots/pi3hat
)

target_link_libraries(Telemetry PUBLIC mjbots_moteus mjbots_pi3hat)