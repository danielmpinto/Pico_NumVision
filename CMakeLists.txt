cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

# Pull in FreeRTOS
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/freertos/FreeRTOS-Kernel)
include(freertos/FreeRTOS_Kernel_import.cmake)


project(camera_example C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()





add_library(camera INTERFACE)

target_include_directories(camera INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/include
	${CMAKE_CURRENT_LIST_DIR}/freertos
)

target_sources(camera INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/src/ov7670/camera.c
	${CMAKE_CURRENT_LIST_DIR}/src/ov7670/format.c
	${CMAKE_CURRENT_LIST_DIR}/src/ov7670/ov7670.c
	${CMAKE_CURRENT_LIST_DIR}/src/st7735/ST7735_TFT.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/st7735/ST7735_TFT_graphics.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/st7735/ST7735_TFT_Print.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/st7735/ST7735_TFT_Font.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/st7735/ST7735_TFT_Bitmap_Data.cpp
)

pico_generate_pio_header(camera ${CMAKE_CURRENT_LIST_DIR}/camera.pio)

target_link_libraries(camera INTERFACE
	hardware_clocks
	hardware_dma
	hardware_irq
	hardware_pio
)

add_subdirectory("example")


# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)