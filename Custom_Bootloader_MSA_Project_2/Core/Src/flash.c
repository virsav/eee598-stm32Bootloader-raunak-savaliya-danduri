
#include "flash.h"

/* pointer specifying the user app jump address*/
typedef void (*fnc_ptr)(void);

/* Write/Flash the memory and return the status after the flashing operation */
flash_status erase_flash(uint32_t addr)
{

  HAL_FLASH_Unlock();

  flash_status status_flash = FLASH_ERROR;
  FLASH_EraseInitTypeDef init_erase;
  uint32_t error = 0u;

  init_erase.TypeErase = FLASH_TYPEERASE_PAGES;
  init_erase.PageAddress = addr;

  /* Specify the number of pages to erase */
  init_erase.NbPages = 10;
  /* Erase the requested sector */
  if (HAL_OK == HAL_FLASHEx_Erase(&init_erase, &error))
  {
    status_flash = FLASH_OK;
  }

  HAL_FLASH_Lock();

  return status_flash;
}

/* Write/Flash the memory and return the status after the flashing operation */
flash_status write_flash(uint32_t addr, uint32_t *data, uint32_t len)
{
  flash_status status_flash = FLASH_OK;

  HAL_FLASH_Unlock();

  /* start the loop */
  for (uint32_t i = 0u; (i < len) && (FLASH_OK == status_flash); i++)
  {
	  /* report error if the specified address is greater than the end address of the flash */
    if (FLASH_APP_END_ADDRESS <= addr)
    {
      status_flash |= FLASH_ERROR_SIZE;
    }
    else
    {
    	/* Start the flashing and report the status  */
      if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data[i]))
      {
        status_flash |= FLASH_ERROR_WRITE;
      }
      /* check the contents of the memory, if wrong report an error */
      if (((data[i])) != (*(volatile uint32_t*)addr))
      {
        status_flash |= FLASH_ERROR_READBACK;
      }

      /* increase the address */
      addr += 4u;
    }
  }

  HAL_FLASH_Lock();

  return status_flash;
}

/* Jump to the user application  */
void flash_jump_userapp(void)
{
	/* define function pointer to store the address of the app's location */
  fnc_ptr jump_to_userapp;
  jump_to_userapp = (fnc_ptr)(*(volatile uint32_t*) (FLASH_APP_START_ADDRESS+4u));
  HAL_DeInit();
  /* Set the main stack pointer to point to the start of the app's address */
  __set_MSP(*(volatile uint32_t*)FLASH_APP_START_ADDRESS);
  jump_to_userapp(); /* jump to app  */
}

