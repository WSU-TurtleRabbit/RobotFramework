add_library(BallDetection detect_ball.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(BallDetection 
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(BallDetection 
    INTERFACE
    ${OpenCV_LIBS})