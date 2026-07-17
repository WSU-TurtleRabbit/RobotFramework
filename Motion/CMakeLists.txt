add_library(Motion
    MotionBridge.cpp
    motion_config_yaml.cpp
    phx/wire.cpp
)

target_include_directories(Motion
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Motion PUBLIC
    yaml-cpp
)
