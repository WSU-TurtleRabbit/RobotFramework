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
    test_controller.cpp
    test_estimator.cpp
    test_kinematics.cpp
    test_matchctrl.cpp
    test_motor_formats.cpp
    test_wire.cpp
    test_executor.cpp
    test_motion_bridge.cpp
    test_safety.cpp
    test_skills.cpp
    test_trajectory.cpp
    test_telemetry_wire.cpp
    ${RF_ROOT}/Networks/decode.cpp
    ${RF_ROOT}/Networks/matchctrl.cpp
    ${RF_ROOT}/Math/kinematics.cpp
    ${RF_ROOT}/Motion/controller.cpp
    ${RF_ROOT}/Motion/estimator.cpp
    ${RF_ROOT}/Motion/skills.cpp
    ${RF_ROOT}/Motion/trajectory.cpp
    ${RF_ROOT}/Motion/phx/wire.cpp
    ${RF_ROOT}/Motion/MotionBridge.cpp
    ${RF_ROOT}/Safety/supervisor.cpp
    ${RF_ROOT}/Telemetry/telemetry_wire.cpp
)

# mjbots/moteus is on the include path for moteus_protocol.h only — it is
# hardware-free (the pi3hat transport headers are NOT, and stay Pi-only).
target_include_directories(host_tests PRIVATE
    ${RF_ROOT}
    ${RF_ROOT}/mjbots/moteus
)

if(MSVC)
    # moteus_multiplex.h guards on the GCC/Clang predefined macro
    # __ORDER_LITTLE_ENDIAN__. MSVC never defines it, but every MSVC target
    # is little-endian, so defining it here is truthful (GCC's value: 1234).
    # _USE_MATH_DEFINES exposes M_PI from MSVC's <cmath> for the same header.
    target_compile_definitions(host_tests PRIVATE
        __ORDER_LITTLE_ENDIAN__=1234
        __BYTE_ORDER__=1234
        _USE_MATH_DEFINES
    )
endif()
