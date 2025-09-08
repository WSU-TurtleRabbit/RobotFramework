cmake_minimum_required(VERSION 3.25)

project(RobotFramework)

find_package(OpenCV REQUIRED)

add_library(COMMON_LIBS INTERFACE)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} RobotFramework.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE COMMON_LIBS)

add_executable(StopMotor StopMotor.cpp)

target_link_libraries(StopMotor PRIVATE COMMON_LIBS)

target_include_directories(COMMON_LIBS INTERFACE
    /usr/include)

target_link_libraries(COMMON_LIBS INTERFACE
    ${OpenCV_LIBS}
    BallDetection 
    Networks 
    wheel_math 
    moteus 
    pi3hat 
    bcm_host 
    )

add_subdirectory(BallDetection)

add_subdirectory(Networks)

add_subdirectory(Math)

add_subdirectory(mjbots/moteus)

add_subdirectory(mjbots/pi3hat)

