add_library(pi3hat pi3hat.cc)

target_include_directories(
    pi3hat
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
    