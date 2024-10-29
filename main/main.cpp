/*!
	@file     main.cpp
	@author   Gavin Lyons
	@brief Example cpp file for ST7735_TFT_PICO library.
			Tests  Hello World
	@note  See USER OPTIONS 0-3 in SETUP function

	@test
		-# Test0 Print out Hello world  

*/


// Sections :: Camera setup
#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "camera.h"
#include "format.h"

#define CAMERA_PIO      pio0
#define CAMERA_BASE_PIN 10
#define CAMERA_XCLK_PIN 21
#define CAMERA_SDA      0
#define CAMERA_SCL      1

// Section ::  libraries 
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "ST7735_TFT.hpp"






static inline int __i2c_write_blocking(void *i2c_handle, uint8_t addr, const uint8_t *src, size_t len)
{
	return i2c_write_blocking((i2c_inst_t *)i2c_handle, addr, src, len, false);
}

static inline int __i2c_read_blocking(void *i2c_handle, uint8_t addr, uint8_t *dst, size_t len)
{
	return i2c_read_blocking((i2c_inst_t *)i2c_handle, addr, dst, len, false);
}

// From http://www.paulbourke.net/dataformats/asciiart/
const char charmap[] = " .:-=+*#%@";

int main() {
    stdio_init_all();

    // Wait some time for USB serial connection
    sleep_ms(3000);

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    i2c_init(i2c0, 100000);
    gpio_set_function(CAMERA_SDA, GPIO_FUNC_I2C);
    gpio_set_function(CAMERA_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(CAMERA_SDA);
    gpio_pull_up(CAMERA_SCL);

    struct camera camera;
    struct camera_platform_config platform = {
        .i2c_write_blocking = __i2c_write_blocking,
        .i2c_read_blocking = __i2c_read_blocking,
        .i2c_handle = i2c0,

        .pio = CAMERA_PIO,
        .xclk_pin = CAMERA_XCLK_PIN,
        .xclk_divider = 9,
        .base_pin = CAMERA_BASE_PIN,
        .base_dma_channel = -1,
    };

    int ret = camera_init(&camera, &platform);
    if (ret) {
        printf("camera_init failed: %d\n", ret);
        return 1;
    }

    const uint16_t width = CAMERA_WIDTH_DIV8;
    const uint16_t height = CAMERA_HEIGHT_DIV8;

    struct camera_buffer *buf = camera_buffer_alloc(FORMAT_YUV422, width, height);
    assert(buf);
    int frame_id = 0;
    while (1) {
        printf("[%03dx%03d] ", width, height);
        gpio_put(LED_PIN, 1);
        ret = camera_capture_blocking(&camera, buf, true);
        gpio_put(LED_PIN, 0);
        if (ret != 0) {
            printf("Capture error: %d\n", ret);
        } else {
            // Loop pelos pixels e imprime em um formato adequado
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    uint8_t v = buf->data[0][buf->strides[0] * y + x];
                    printf("%d ", v);
                }
            }
            printf("$\n"); // Indica o final da imagem
            frame_id++;
            if (frame_id >= 1000)
                frame_id = 0;
        }

        sleep_ms(5000);
    }
}







// // Section :: Defines   
// //  Test timing related defines 
// #define TEST_DELAY1 1000 // mS
// #define TEST_DELAY2 2000 // mS
// #define TEST_DELAY5 5000 // mS

// // Section :: Globals 
// ST7735_TFT myTFT;

// //  Section ::  Function Headers 

// void Setup(void);  // setup + user options
// void Test0(void);  
// void EndTests(void);

// //  Section ::  MAIN loop

// int main(void) 
// {
// 	Setup();
// 	Test0();
// 	EndTests();
// }
// // *** End OF MAIN **

// //  Section ::  Function Space 

// /*!
// 	@brief setup the TFT :: user options 0-3
// */
// void Setup(void)
// {
// 	stdio_init_all(); // optional for error messages , Initialize chosen serial port, default 38400 baud
// 	TFT_MILLISEC_DELAY(TEST_DELAY1);
// 	printf("TFT :: Start\r\n");
	
// //*************** USER OPTION 0 SPI_SPEED + TYPE ***********
// 	bool bhardwareSPI = true; // true for hardware spi, false for software
	
// 	if (bhardwareSPI == true) { // hw spi
// 		uint32_t TFT_SCLK_FREQ =  8000 ; // Spi freq in KiloHertz , 1000 = 1Mhz
// 		myTFT.TFTInitSPIType(TFT_SCLK_FREQ, spi0); 
// 	} else { // sw spi
// 		uint16_t SWSPICommDelay = 0; // optional SW SPI GPIO delay in uS
// 		myTFT.TFTInitSPIType(SWSPICommDelay);
// 	}
// //*********************************************************
// // ******** USER OPTION 1 GPIO *********
// // NOTE if using Hardware SPI clock and data pins will be tied to 
// // the chosen interface eg Spi0 CLK=18 DIN=19)
// 	int8_t SDIN_TFT = 19; 
// 	int8_t SCLK_TFT = 18; 
// 	int8_t DC_TFT = 3;
// 	int8_t CS_TFT = 2 ;  
// 	int8_t RST_TFT = 17;
// 	myTFT.TFTSetupGPIO(RST_TFT, DC_TFT, CS_TFT, SCLK_TFT, SDIN_TFT);
// //**********************************************************

// // ****** USER OPTION 2 Screen Setup ****** 
// 	uint8_t OFFSET_COL = 0;  // 2, These offsets can be adjusted for any issues->
// 	uint8_t OFFSET_ROW = 0; // 3, with screen manufacture tolerance/defects
// 	uint16_t TFT_WIDTH = 128;// Screen width in pixels
// 	uint16_t TFT_HEIGHT = 128; // Screen height in pixels
// 	myTFT.TFTInitScreenSize(OFFSET_COL, OFFSET_ROW , TFT_WIDTH , TFT_HEIGHT);
// // ******************************************

// // ******** USER OPTION 3 PCB_TYPE  **************************
// 	myTFT.TFTInitPCBType(myTFT.TFT_ST7735R_Red); // pass enum,4 choices,see README
// //**********************************************************
// }

// /*!
// 	@brief print out hello world on TFT
// */
// void Test0(void) {

// 	char teststr1[] = "Hello";
// 	char teststr2[] = "World";
	
// 	myTFT.TFTfillScreen(ST7735_BLUE);
// 	myTFT.TFTFontNum(myTFT.TFTFont_Default);
// 	myTFT.TFTdrawText(15, 15, teststr1, ST7735_WHITE, ST7735_BLACK, 2);
// 	myTFT.TFTdrawRectWH(80,80,20,20,0xFF);
// 	myTFT.TFTdrawText(15, 35, teststr2, ST7735_WHITE, ST7735_BLACK, 2);
// 	TFT_MILLISEC_DELAY(TEST_DELAY5);
// 	TFT_MILLISEC_DELAY(TEST_DELAY2);
// 	myTFT.TFTfillScreen(ST7735_BLACK);
// 	TFT_MILLISEC_DELAY(TEST_DELAY1);
// }

// /*!
// 	@brief  Stop testing and shutdown the TFT
// */
// void EndTests(void)
// {
// 	myTFT.TFTPowerDown(); 
// 	printf("TFT :: Tests Over");
// }

// // *************** EOF ****************




