add_library(Arduino Arduino.cpp)

find_package(OpenCV REQUIRED)

target_include_directories(Arduino 
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(Arduino 
    INTERFACE
    ${OpenCV_LIBS})