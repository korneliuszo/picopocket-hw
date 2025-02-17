/*
 * board.h
 *
 *  Created on: 11 sty 2024
 *      Author: nyuu
 */

#ifndef FW_BOARD_H_
#define FW_BOARD_H_

// setting first overrides the value in the default header
#define PICO_FLASH_SPI_CLKDIV 6

#define PSRAM_PIN_CS 13
#define PSRAM_PIN_SCK 14
#define PSRAM_PIN_MOSI 15
#define PSRAM_PIN_MISO 12
#define PSRAM_PIO 1

#define TOUCH_PIN_CS 9
#define TOUCH_PIN_SCK 14
#define TOUCH_PIN_MOSI 15
#define TOUCH_PIN_MISO 12
#define TOUCH_PIO 1

#define AUDIO_DIN 16
#define AUDIO_BCK 17
#define AUDIO_LRCK 18
#define AUDIO_PDM 10
#define AUDIO_PIO 1
#define AUDIO_DMA_IRQN 0
#define AUDIO_PIO_IRQN 0

#define CYW43_PIO_CLOCK_DIV 3

// pick up the rest of the settings
#include "boards/pico_w.h"


#endif /* FW_BOARD_H_ */
