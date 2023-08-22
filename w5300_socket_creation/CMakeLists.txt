cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)



project(w5300_send_tcp)

pico_sdk_init()

add_executable(w5300_send_tcp
    main.c dhcp.c socket.c w5300.c wizchip_conf.c 
)

pico_add_extra_outputs(w5300_send_tcp)

target_link_libraries(w5300_send_tcp pico_stdlib hardware_gpio)


# Disable usb output, Enable uart output
pico_enable_stdio_usb(w5300_send_tcp 0)
pico_enable_stdio_uart(w5300_send_tcp 1)
