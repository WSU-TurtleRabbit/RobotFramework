add_library(detect_ball detect_ball.cpp)

target_include_directories(detect_ball 
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
    