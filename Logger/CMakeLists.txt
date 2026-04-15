add_library(Logger Logger.cpp)

target_include_directories(Logger
    INTERFACE
    ${CMAKE_SOURCE_DIR}
)
