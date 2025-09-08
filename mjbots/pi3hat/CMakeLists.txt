add_library(pi3hat pi3hat.cc)

find_library(BCM_HOST_LIB bcm_host)

target_link_libraries(pi3hat PUBLIC ${BCM_HOST_LIB})

target_include_directories(
    pi3hat
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    )
    
