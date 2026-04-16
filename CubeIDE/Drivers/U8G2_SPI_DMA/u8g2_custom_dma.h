/*
 * u8g2_custom_dma.h
 *
 *  Created on: Sep 23, 2025
 *      Author: Carlo
 */

#ifndef INC_U8G2_CUSTOM_DMA_H_
#define INC_U8G2_CUSTOM_DMA_H_

#pragma once

#include "main.h"
#include "u8g2.h"

extern SPI_HandleTypeDef *LCD_SPI;

// buffer size must be large enough for one screen (~2.5 KB for ST7920)
#define DMA_12864_BUFFER_SIZE_MAX  2570

// DMA state
extern volatile uint8_t spi_dma_busy;

// staging buffer
extern uint8_t dma_send_buffer[DMA_12864_BUFFER_SIZE_MAX];
extern uint16_t dma_send_buffer_counter;

// callbacks


uint8_t u8x8_spi_dma(u8x8_t *u8x8, uint8_t msg,
                               uint8_t arg_int, void *arg_ptr);

uint8_t u8x8_stm32_gpio_and_delay_cb(u8x8_t *u8x8,
                                     uint8_t msg,
                                     uint8_t arg_int,
                                     void *arg_ptr);

// init + reset helpers
void reset_lcd(void);
void setup_u8g2_st7920_SPI(u8g2_t *u8g2, SPI_HandleTypeDef *dispSPI);


#endif /* INC_U8G2_CUSTOM_DMA_H_ */
