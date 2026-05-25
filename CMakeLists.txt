cmake_minimum_required(VERSION 3.25)

project(RobotFramework)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Interface library for common dependencies
add_library(COMMON_LIBS INTERFACE)

target_include_directories(COMMON_LIBS INTERFACE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(COMMON_LIBS INTERFACE
    ${OpenCV_LIBS}
    BallDetection
    Arduino
    Networks 
    wheel_math
    moteus 
    pi3hat 
    Telemetry
    Logger
    atomic 
    yaml-cpp
)

# Function to build an executable with common dependencies
function(build_executable target_name source_file)
       add_executable(${target_name} ${source_file})
       target_link_libraries(${target_name} PRIVATE COMMON_LIBS)
   endfunction()

# Subdirectories
add_subdirectory(mjbots/moteus)
add_subdirectory(mjbots/pi3hat)
add_subdirectory(BallDetection)
add_subdirectory(Arduino)
add_subdirectory(Networks)
add_subdirectory(Math)
add_subdirectory(Telemetry)
add_subdirectory(Logger)

# Build the main executable
build_executable(RobotFramework RobotFramework.cpp)
build_executable(Motor_test tests/MultiMotor.cpp)
build_executable(pi3hat_tool mjbots/pi3hat/pi3hat_tool.cc)

