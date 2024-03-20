pico_add_library(pico_pca9535)

target_sources(pico_pca9535 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/pico_pca9535.cpp
        )

target_include_directories(pico_pca9535 INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/include)

pico_mirrored_target_link_libraries(pico_pca9535 INTERFACE
        pico_stdlib
        hardware_i2c
        pico_async_context_freertos
        )
