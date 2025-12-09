add_library(Logger
    Logger.cpp
)

target_include_directories(Logger
    PUBLIC
        ${CMAKE_SOURCE_DIR}
)
