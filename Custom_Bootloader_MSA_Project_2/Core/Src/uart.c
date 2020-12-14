
#include "uart.h"

/* Receive the data through UART */
uart_status uart_rcv(uint8_t *data, uint16_t len)
{
  uart_status status_uart = UART_ERROR;

  if (HAL_OK == HAL_UART_Receive(&huart3, data, len, UART_TIMEOUT))
  {
    status_uart = UART_OK;
  }

  return status_uart;
}

/* Send the string data through UART */
uart_status uart_trnsmt_str(uint8_t *data)
{
  uart_status status_uart = UART_ERROR;
  uint16_t len = 0u;

  /* Find the length of data to be transmitted */
  while ('\0' != data[len])
  {
    len++;
  }

  if (HAL_OK == HAL_UART_Transmit(&huart3, data, len, UART_TIMEOUT))
  {
    status_uart = UART_OK;
  }

  return status_uart;
}

/* send a single chracter through UART*/
uart_status uart_trnsmt_ch(uint8_t data)
{
  uart_status status_uart = UART_ERROR;

  /* Get the state of the UART module */
  if (HAL_UART_STATE_TIMEOUT == HAL_UART_GetState(&huart3))
  {
    HAL_UART_Abort(&huart3);
  }

  if (HAL_OK == HAL_UART_Transmit(&huart3, &data, 1u, UART_TIMEOUT))
  {
    status_uart = UART_OK;
  }
  return status_uart;
}
