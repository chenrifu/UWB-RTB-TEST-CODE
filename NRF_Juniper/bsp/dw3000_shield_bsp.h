/*
 * @attention
 *
 * Copyright 2019 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#ifndef _DW3000_SHIELD_BSP_H
#define _DW3000_SHIELD_BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/* ENABLE_USB_PRINT Macro is uncommented then Segger RTT Print will be enabled*/
#define ENABLE_USB_PRINTx

#if defined(BOARD_PCA10040)
#include "pca10040.h"
#elif defined(BOARD_PCA10056)
#include "pca10056.h"
#endif

#define DW3000_RST_Pin      ARDUINO_7_PIN
#define DW3000_IRQ_Pin      ARDUINO_8_PIN
#define DW3000_WUP_Pin      ARDUINO_9_PIN

// SPI defs
#define DW3000_CS_Pin       ARDUINO_10_PIN
#define DW3000_CLK_Pin      ARDUINO_13_PIN  // DWM3000 shield SPIM1 sck connected to DW1000
#define DW3000_MOSI_Pin     ARDUINO_11_PIN  // DWM3000 shield SPIM1 mosi connected to DW1000
#define DW3000_MISO_Pin     ARDUINO_12_PIN  // DWM3000 shield SPIM1 miso connected to DW1000
#define DW3000_SPI_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW // 

// UART symbolic constants
#define UART_0_TX_PIN       TX_PIN_NUMBER           // DWM1001 module pin 20, DEV board name RXD
#define UART_0_RX_PIN       RX_PIN_NUMBER           // DWM1001 module pin 18, DEV board name TXD
#define DW3000_RTS_PIN_NUM      UART_PIN_DISCONNECTED
#define DW3000_CTS_PIN_NUM      UART_PIN_DISCONNECTED

#ifdef __cplusplus
}
#endif
#endif // _DW3000_BSP_H
