/**
 * Copyright (c) 2022 Brian Starkey <stark3y@gmail.com>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/stdio.h"
#include "pico/stdlib.h"

#include "camera/camera.h"
#include "camera/format.h"

#define CAMERA_PIO      pio0
#define CAMERA_BASE_PIN 10
#define CAMERA_XCLK_PIN 21
#define CAMERA_SDA      0
#define CAMERA_SCL      1

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



#include "hardware/spi.h"

// Definindo os pinos
#define CS_PIN   17  // Chip Select
#define DC_PIN   12  // Data/Command
#define RST_PIN  15  // Reset
#define SPI_SCK  14  // Clock
#define SPI_MOSI 13  // Data

// Comandos do ST7735 (Exemplo, ajuste conforme necessário)
#define ST7735_NOP          0x00
#define ST7735_SWRESET      0x01
#define ST7735_SLPOUT       0x11
#define ST7735_INVOFF       0x20
#define ST7735_INVON        0x21
#define ST7735_CASET        0x2A
#define ST7735_RASET        0x2B
#define ST7735_RAMWR        0x2C
#define ST7735_MADCTL       0x36
#define ST7735_COLMOD       0x3A
#define ST7735_DISPON       0x29

// Inicializa o SPI 1
void init_spi() {
    spi_init(spi1, 1000000); // Inicializa SPI a 1 MHz
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI); // Configura SCK para SPI1
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI); // Configura MOSI para SPI1
    gpio_init(CS_PIN); // Inicializa CS
    gpio_init(DC_PIN); // Inicializa DC
    gpio_init(RST_PIN); // Inicializa RST

    gpio_set_dir(CS_PIN, GPIO_OUT); // Configura CS como saída
    gpio_set_dir(DC_PIN, GPIO_OUT); // Configura DC como saída
    gpio_set_dir(RST_PIN, GPIO_OUT); // Configura RST como saída
}

// Envia um comando para o display
void sendCommand(uint8_t cmd) {
    gpio_put(CS_PIN, 0); // Ativa o chip select
    gpio_put(DC_PIN, 0); // Envia comando
    spi_write_blocking(spi1, &cmd, 1); // Envia o comando
    gpio_put(CS_PIN, 1); // Desativa o chip select
}

// Envia dados para o display
void sendData(uint8_t data) {
    gpio_put(CS_PIN, 0); // Ativa o chip select
    gpio_put(DC_PIN, 1); // Envia dados
    spi_write_blocking(spi1, &data, 1); // Envia os dados
    gpio_put(CS_PIN, 1); // Desativa o chip select
}

// Envia um comando seguido de dados
void sendCommandWithData(uint8_t cmd, uint8_t *data, size_t length) {
    sendCommand(cmd);
    gpio_put(CS_PIN, 0); // Ativa o chip select
    gpio_put(DC_PIN, 1); // Envia dados
    spi_write_blocking(spi1, data, length); // Envia os dados
    gpio_put(CS_PIN, 1); // Desativa o chip select
}

// Inicializa o display
void initDisplay() {
    gpio_put(RST_PIN, 1); // Define RST alto
    sleep_ms(100);
    gpio_put(RST_PIN, 0); // Define RST baixo
    sleep_ms(100);
    gpio_put(RST_PIN, 1); // Define RST alto
    sleep_ms(100);

    sendCommand(ST7735_SWRESET); // Reinicializa o display
    sleep_ms(150);
    sendCommand(ST7735_SLPOUT); // Sai do modo de suspensão
    sleep_ms(150);
    sendCommand(ST7735_COLMOD); // Define o formato de cor
    uint8_t colmod = 0x05; // 16 bits por pixel
    sendData(colmod);
    sendCommand(ST7735_MADCTL); // Define a orientação do display
    uint8_t madctl = 0xC8; // Exemplo de configuração
    sendData(madctl);
    sendCommand(ST7735_DISPON); // Liga o display
}

// Define um pixel no display
void setPixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= 128 || y < 0 || y >= 160) return; // Verifica os limites

    // Define a área onde o pixel será desenhado
    uint8_t col[4] = {0x2A, 0x00, (uint8_t)(x >> 8), (uint8_t)(x & 0xFF)};
    uint8_t row[4] = {0x2B, 0x00, (uint8_t)(y >> 8), (uint8_t)(y & 0xFF)};
    sendCommandWithData(ST7735_CASET, col, sizeof(col));
    sendCommandWithData(ST7735_RASET, row, sizeof(row));
    
    // Envia o dado do pixel
    sendCommand(ST7735_RAMWR);
    sendData(color >> 8); // Parte alta
    sendData(color & 0xFF); // Parte baixa
}

int main() {
    stdio_init_all();
    init_spi();
    initDisplay();

    // Desenha um pixel vermelho no centro
    setPixel(64, 80, 0x0000); // 0xF800 é a cor vermelha em formato RGB565

    while (true) {
        // Loop principal
    }
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
		printf("[%03dx%03d] %04d$", width, height, frame_id);
		gpio_put(LED_PIN, 1);
		ret = camera_capture_blocking(&camera, buf, true);
		gpio_put(LED_PIN, 0);
		if (ret != 0) {
			printf("Capture error: %d\n", ret);
		} else {
			// printf("Capture success\n");
			int y, x;
			for (y = 0; y < height; y++) {
				char row[width];
				for (x = 0; x < width; x++) {
					uint8_t v = buf->data[0][buf->strides[0] * y + x];
					char snum[4];
    				int n = sprintf(snum, "%d", v);
					printf(" %s", snum);
				}
			}
			printf("\n");
			frame_id++;
			if (frame_id >= 1000)
				frame_id = 0;
		}

		sleep_ms(5000);
	}
}
