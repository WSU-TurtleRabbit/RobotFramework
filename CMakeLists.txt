cmake_minimum_required(VERSION 3.25)

project(RobotFramework)

find_package(OpenCV REQUIRED)

# Interface library for common dependencies
add_library(COMMON_LIBS INTERFACE)

target_include_directories(COMMON_LIBS INTERFACE
    ${OpenCV_INCLUDE_DIRS}
    /usr/include
)

target_link_libraries(COMMON_LIBS INTERFACE
    ${OpenCV_LIBS}
    Arduino
    BallDetection 
    Networks 
    wheel_math
    moteus 
    pi3hat 
    bcm_host
    MotorControl
)

# Function to build an executable with common dependencies
function(build_executable file_name)
    add_executable(${file_name} ${file_name}.cpp)
    target_link_libraries(${file_name} PRIVATE COMMON_LIBS)
endfunction()

# Subdirectories
add_subdirectory(mjbots/moteus)
add_subdirectory(mjbots/pi3hat)
add_subdirectory(Arduino)
add_subdirectory(BallDetection)
add_subdirectory(Networks)
add_subdirectory(Math)
add_subdirectory(MotorControl)

# Build the main executable
build_executable(RobotFramework)
build_executable(StopMotor)
