add_library(Motion
    skills.cpp
    estimator.cpp
    trajectory.cpp
    controller.cpp
    actuators.cpp
    match_bridge.cpp
    match_config_yaml.cpp
)

target_include_directories(Motion
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Motion PUBLIC
    yaml-cpp
)
