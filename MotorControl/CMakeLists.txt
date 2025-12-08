add_library(MotorControl MotorControl.cpp)

target_include_directories(MotorControl
    INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MotorControl PUBLIC
    moteus 
    pi3hat
)
