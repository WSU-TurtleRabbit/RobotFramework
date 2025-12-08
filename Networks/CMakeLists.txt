add_library(Networks decode.cpp UDP.cpp)

target_include_directories(Networks
    INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
