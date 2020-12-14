
#ifndef UART_H_
#define UART_H_

#include "stm32f3xx_hal.h"

extern UART_HandleTypeDef huart3;

/* HAL UART timeout defined */
#define UART_TIMEOUT ((uint16_t)1000u)

/* Define the status constants for UART communication */
typedef enum {
  UART_OK     = 0x00u, /*Successful operation */
  UART_ERROR  = 0xFFu  /* Error */
} uart_status;

uart_status uart_rcv(uint8_t *data, uint16_t length);
uart_status uart_trnsmt_str(uint8_t *data);
uart_status uart_trnsmt_ch(uint8_t data);


#endif /* UART_H_ */
