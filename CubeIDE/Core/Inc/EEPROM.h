#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"

HAL_StatusTypeDef EEPROM_Init();
HAL_StatusTypeDef EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
HAL_StatusTypeDef EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size);
HAL_StatusTypeDef EEPROM_PageErase (uint16_t page);

#endif /* INC_EEPROM_H_ */
