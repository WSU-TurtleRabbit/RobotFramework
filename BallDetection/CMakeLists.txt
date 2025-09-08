add_library(BallDetection detect_ball.cpp)

target_include_directories(BallDetection 
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})
    