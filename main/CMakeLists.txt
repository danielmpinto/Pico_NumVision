cmake_minimum_required(VERSION 3.12)
# rest of your project
add_executable(pico_numvision
    main.c 
)
pico_enable_stdio_usb(pico_numvision 1)
# target_include_directories(pico_numvision PRIVATE ${CMAKE_CURRENT_LIST_DIR}/.. include/camera)
# add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/.. camera)
target_link_libraries(pico_numvision PRIVATE camera pico_stdlib hardware_i2c hardware_gpio hardware_spi FreeRTOS-Kernel FreeRTOS-Kernel-Heap4) 
# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_numvision)
