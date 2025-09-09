add_library(wheel_math wheel_math.cpp)

target_include_directories(wheel_math 
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
    