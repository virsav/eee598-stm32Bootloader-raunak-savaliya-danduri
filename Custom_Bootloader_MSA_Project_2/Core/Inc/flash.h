
#ifndef FLASH_H_
#define FLASH_H_

#include "stm32f3xx_hal.h"
extern  uint32_t *userAppAddr;

/* define the start and end address of the flash for the app  */

#define FLASH_APP_START_ADDRESS ((uint32_t)(*userAppAddr))
#define FLASH_APP_END_ADDRESS   (((uint32_t)(*userAppAddr + ((uint32_t)0x800)))*17) /**< Leave a little extra space at the end. */

/* Define the status constants for the flashing functions */

typedef enum {
  FLASH_OK              = 0x00u, /* Successful operation */
  FLASH_ERROR_SIZE      = 0x01u, /* Too large size of the flash*/
  FLASH_ERROR_WRITE     = 0x02u, /* Flashing failed */
  FLASH_ERROR_READBACK  = 0x04u, /* Successful flashing, but wrong contents in the memory*/
  FLASH_ERROR           = 0xFFu  /* Error */
} flash_status;

flash_status erase_flash(uint32_t address);
flash_status write_flash(uint32_t address, uint32_t *data, uint32_t length);
void flash_jump_userapp(void);

#endif /* FLASH_H_ */
