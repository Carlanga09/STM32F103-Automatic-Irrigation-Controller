/*
 * u8g2_st7920_dma.c
 *
 *  Created on: Sep 23, 2025
 *      Author: Carlos David Cancino Bejarano
 */

#include "u8g2_custom_dma.h"
#include <string.h>

volatile uint8_t spi_dma_busy = 0;
uint8_t dma_send_buffer[DMA_12864_BUFFER_SIZE_MAX];
uint16_t dma_send_buffer_counter = 0;
SPI_HandleTypeDef *LCD_SPI = NULL;

// ----------------- DMA COMPLETE CALLBACK -----------------
void U8G2_SPI_DMA_COMPLETE() {
	spi_dma_busy = 0;
	dma_send_buffer_counter = 0;
}

// ----------------- INTERNAL HELPERS -----------------
static inline uint8_t check_buffer_space(uint8_t new_len) {
	return (dma_send_buffer_counter + new_len) <= DMA_12864_BUFFER_SIZE_MAX;
}

static inline void flush_dma_buffer(void) {
	if (dma_send_buffer_counter > 0) {
		spi_dma_busy = 1;
		HAL_SPI_Transmit_DMA(LCD_SPI, dma_send_buffer, dma_send_buffer_counter);
		// next chunk will wait until HAL_SPI_TxCpltCallback resets busy flag
	}
}

// ----------------- U8G2 GPIO+DELAY CALLBACK -----------------
uint8_t u8x8_stm32_gpio_and_delay_cb(u8x8_t *u8x8,
		uint8_t msg,
		uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		break;
	case U8X8_MSG_DELAY_NANO:
		__NOP();
		break;
	case U8X8_MSG_DELAY_MILLI:
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_DC:
		// ST7920 has no DC pin
		break;
	case U8X8_MSG_GPIO_RESET:
		// handled manually
		break;
	default:
		break;
	}
	return 1;
}

// ----------------- U8G2 BYTE CALLBACK -----------------
uint8_t u8x8_spi_dma(u8x8_t *u8x8,
		uint8_t msg,
		uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		// wait for DMA free if no space
		while (!check_buffer_space(arg_int) || spi_dma_busy);

		memcpy(&dma_send_buffer[dma_send_buffer_counter],
				(uint8_t *)arg_ptr, arg_int);
		dma_send_buffer_counter += arg_int;
		break;

	case U8X8_MSG_BYTE_START_TRANSFER:
		// no CS control needed for ST7920 in SPI mode
		break;

	case U8X8_MSG_BYTE_END_TRANSFER:
		flush_dma_buffer();
		break;

	case U8X8_MSG_BYTE_INIT:
		// nothing extra
		break;

	default:
		return 0;
	}
	return 1;
}

// ----------------- LCD RESET & SETUP -----------------

void setup_u8g2_st7920_SPI(u8g2_t *u8g2, SPI_HandleTypeDef *dispSPI) {
    LCD_SPI=dispSPI;

    // Register SPI callback
    HAL_SPI_RegisterCallback(dispSPI, HAL_SPI_TX_COMPLETE_CB_ID, U8G2_SPI_DMA_COMPLETE);
    u8g2_Setup_st7920_s_128x64_f(u8g2, U8G2_R0, u8x8_spi_dma, u8x8_stm32_gpio_and_delay_cb);
    u8g2_InitDisplay(u8g2);
    u8g2_SetPowerSave(u8g2, 0);
    u8g2_ClearBuffer(u8g2);
}

