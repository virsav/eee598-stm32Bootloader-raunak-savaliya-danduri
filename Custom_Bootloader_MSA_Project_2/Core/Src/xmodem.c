
#include "xmodem.h"

/* Global variables. */
static uint8_t xmodem_packet_number = 1u;         /* variable to count the packets */
static uint32_t xmodem_actual_flash_address = 0u; /* Flashing address */
static uint8_t x_first_packet_received = false;   /* variable to check the first packet received */

/* Local functions. */
static uint16_t xmodem_calculate_crc(uint8_t *data, uint16_t len);  /* function to calculate crc value */
static xmodem_status xmodem_packet_handler(uint8_t size);   /* function to take the packet and write to the flash  */
static xmodem_status xmodem_error_handler(uint8_t *error_num, uint8_t max_error_num);   /* function to handle the error */

/* Receive the binary through XMODEM protocol */
void xmodem_rcv(void)
{
  volatile xmodem_status status = X_OK;
  uint8_t error_number = 0u;

  x_first_packet_received = false;
  xmodem_packet_number = 1u;
  xmodem_actual_flash_address = FLASH_APP_START_ADDRESS;

  /* Keep looping till there is an error or until there is an application jump */
  while (X_OK == status)
  {
    uint8_t header = 0x00u;

    /* receive the header through UART */
    uart_status comm_status = uart_rcv(&header, 1u);

    /* notify the sender that we would use CRC-16 check */
    if ((UART_OK != comm_status) && (false == x_first_packet_received))
    {
      (void)uart_trnsmt_ch(X_C);
    }
    /* handle the error if there is some timeout or any other errors */
    else if ((UART_OK != comm_status) && (true == x_first_packet_received))
    {
      status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
    }
    else
    {
    	/* Blank */
    }
    xmodem_status packet_status = X_ERROR;
    /* Check for headers: SOH, STX, EOT and CAN. */
    switch(header)
    {

      /* 128 or 1024 bytes of data. */
      case X_SOH: /* Blank */
      case X_STX:
    	  /* send acknowledge if packet handled successfully*/
        packet_status = xmodem_packet_handler(header);
        if (X_OK == packet_status)
        {
          uart_trnsmt_ch(X_ACK);
        }
        /* handle flash related errors and set the error number to X_MAX_ERRORS */
        else if (X_ERROR_FLASH == packet_status)
        {
          error_number = X_MAX_ERRORS;
          status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
        }
        /* handle other errors */
        else
        {
          status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
        }
        break;
        /* X_EOT signifies end of transmission */
      case X_EOT:
    	  /* Send acknowledge to user to specify end and then jump to app */
        (void)uart_trnsmt_ch(X_ACK);
        (void)uart_trnsmt_str((uint8_t*)"\n\rFirmware updated!\n\r");
        (void)uart_trnsmt_str((uint8_t*)"Jumping to user application...\n\r");
        flash_jump_userapp();
        break;
        /* Abort if error */
      case X_CAN:
        status = X_ERROR;
        break;
      default:
    	  /* Incorrect header received*/
        if (UART_OK == comm_status)
        {
          status = xmodem_error_handler(&error_number, X_MAX_ERRORS);
        }
        break;
    }
  }
}

/* Calculate the CRC-16 value for the input */
static uint16_t xmodem_calculate_crc(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0u;
    while (len)
    {
        len--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

/* Receive packet and write it to the flash */
static xmodem_status xmodem_packet_handler(uint8_t header)
{
  xmodem_status status = X_OK;
  uint16_t size = 0u;

  /* 2 bytes for packet number, 1024 for data, 2 for CRC*/
  uint8_t received_packet_number[X_PACKET_NUMBER_SIZE];
  uint8_t received_packet_data[X_PACKET_1024_SIZE];
  uint8_t received_packet_crc[X_PACKET_CRC_SIZE];

  /* receive the size of the data. */
  if (X_SOH == header)
  {
    size = X_PACKET_128_SIZE;
  }
  else if (X_STX == header)
  {
    size = X_PACKET_1024_SIZE;
  }
  else
  {
	  /* change the staus to error if wrong header*/
    status |= X_ERROR;
  }

  uart_status comm_status = UART_OK;
  /* Receive the packet number, data and CRC from UART. */
  comm_status |= uart_rcv(&received_packet_number[0u], X_PACKET_NUMBER_SIZE);
  comm_status |= uart_rcv(&received_packet_data[0u], size);
  comm_status |= uart_rcv(&received_packet_crc[0u], X_PACKET_CRC_SIZE);
  /* Combine the CRC two bytes */
  uint16_t crc_received = ((uint16_t)received_packet_crc[X_PACKET_CRC_HIGH_INDEX] << 8u) | ((uint16_t)received_packet_crc[X_PACKET_CRC_LOW_INDEX]);
  /* calculate the CRC bytes here too */
  uint16_t crc_calculated = xmodem_calculate_crc(&received_packet_data[0u], size);

  /* if communication error, set the error status*/
  if (UART_OK != comm_status)
  {
    status |= X_ERROR_UART;
  }

  /* check if it is the first packet and if true erase the memory to receive the app data  */
  if ((X_OK == status) && (false == x_first_packet_received))
  {
    if (FLASH_OK == erase_flash(FLASH_APP_START_ADDRESS))
    {
      x_first_packet_received = true;
    }
    else
    {
      status |= X_ERROR_FLASH;
    }
  }

  /* start the error handling and flashing. */
  if (X_OK == status)
  {
    if (xmodem_packet_number != received_packet_number[0u])
    {
    	/* Report error if there is packet number counter mismatch */
      status |= X_ERROR_NUMBER;
    }
    if (255u != (received_packet_number[X_PACKET_NUMBER_INDEX] + received_packet_number[X_PACKET_NUMBER_COMPLEMENT_INDEX]))
    {
    	/* The sum of the packet number and packet number complement aren't 255. Report error */
      status |= X_ERROR_NUMBER;
    }
    if (crc_calculated != crc_received)
    {
    	/* Report error if the calculated and received CRC are not same. */
      status |= X_ERROR_CRC;
    }
  }

  /* Start the flashing if no errors */
    if ((X_OK == status) && (FLASH_OK != write_flash(xmodem_actual_flash_address, (uint32_t*)&received_packet_data[0u], (uint32_t)size/4u)))
    {
    	/* Report error if flashing problem */
      status |= X_ERROR_FLASH;
    }

    /* Increase the packet number and the address counters, if no errors found. */
  if (X_OK == status)
  {
    xmodem_packet_number++;
    xmodem_actual_flash_address += size;
  }

  return status;
}

/* Function to handle errors when receving the binary through XMODEM protocol */
static xmodem_status xmodem_error_handler(uint8_t *error_num, uint8_t max_error_num)
{
  xmodem_status status_xmodem = X_OK;
  /* Increment the error counter*/
  (*error_num)++;
  /* abort, if maximum value reached */
  if ((*error_num) >= max_error_num)
  {
    (void)uart_trnsmt_ch(X_CAN);
    (void)uart_trnsmt_ch(X_CAN);
    status_xmodem = X_ERROR;
  }
  /* Else, send a NAK for a repeat. */
  else
  {
    (void)uart_trnsmt_ch(X_NAK);
    status_xmodem = X_OK;
  }
  return status_xmodem;
}
