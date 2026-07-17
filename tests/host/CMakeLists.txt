# Host-side unit tests for the pure logic modules — no Pi dependencies
# (no pi3hat, no OpenCV, no libserial, no yaml-cpp). Build and run on any
# desktop toolchain:
#
#   cmake -S tests/host -B tests/host/build
#   cmake --build tests/host/build
#   ./tests/host/build/host_tests
#
# Keep this list in sync as pure modules grow; anything that needs hardware
# stays out and is exercised on the robot instead.
cmake_minimum_required(VERSION 3.16)

project(RobotFrameworkHostTests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(RF_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(host_tests
    main.cpp
    test_decode.cpp
    test_kinematics.cpp
    ${RF_ROOT}/Networks/decode.cpp
    ${RF_ROOT}/Math/kinematics.cpp
)

target_include_directories(host_tests PRIVATE ${RF_ROOT})
