add_library(Safety supervisor.cpp)

target_include_directories(Safety
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
