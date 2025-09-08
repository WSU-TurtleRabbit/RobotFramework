project(RobotFrameWork)
add_executable(${PROJECT_NAME} RobotFrameWork.cpp)

add_subdirectory(BallDetection)

add_subdirectory(Networks)

add_subdirectory(Math)

add_subdirectory(mjbots/moteus)

add_subdirectory(mjbots/pi3hat)
