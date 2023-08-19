cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)



project(w5300_test)

pico_sdk_init()

add_executable(w5300_test
    main.c
)

pico_add_extra_outputs(w5300_test)

target_link_libraries(w5300_test pico_stdlib hardware_gpio)


# Disable usb output, Enable uart output
pico_enable_stdio_usb(w5300_test 0)
pico_enable_stdio_uart(w5300_test 1)
