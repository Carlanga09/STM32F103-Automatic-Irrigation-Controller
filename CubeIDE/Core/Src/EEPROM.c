#include "EEPROM.h"
#include "math.h"
#include "string.h"

// Define the I2C
extern I2C_HandleTypeDef hi2c2;
#define EEPROM_I2C &hi2c2

// EEPROM ADDRESS (8bits)
#define EEPROM_ADDR 0x50 << 1 // A0:GND, A1:GND, A2:GND

// Define the Page Size and number of pages
#define PAGE_SIZE 64     // in Bytes
#define PAGE_NUM  512    // number of pages
// Find out the number of bit, where the page addressing starts
const int paddrposition = log(PAGE_SIZE)/log(2);

/*****************************************************************************************************************************************/
uint8_t bytes_temp[4];

// function to determine the remaining bytes
uint16_t bytestowrite (uint16_t size, uint16_t offset)
{
	if ((size+offset)<PAGE_SIZE) return size;
	else return PAGE_SIZE-offset;
}


HAL_StatusTypeDef EEPROM_Init() {
	return HAL_I2C_IsDeviceReady(EEPROM_I2C, EEPROM_ADDR, 1, HAL_MAX_DELAY);
}
/* write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
HAL_StatusTypeDef EEPROM_Write (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size) {
	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	for (int i=0; i<numofpages; i++)
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);  // calculate the remaining bytes to be written

		if (HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000) != HAL_OK) {  // write the data to the EEPROM
			return HAL_ERROR;
		}

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
	}

	return HAL_OK;
}

/* READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to PAGE_NUM-1
 * @offset is the start byte offset in the page. Range from 0 to PAGE_SIZE-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 */
HAL_StatusTypeDef EEPROM_Read (uint16_t page, uint16_t offset, uint8_t *data, uint16_t size) {
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/PAGE_SIZE);

	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	for (int i=0; i<numofpages; i++)
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		if (HAL_I2C_Mem_Read(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, &data[pos], bytesremaining, 1000) != HAL_OK) {
			return HAL_ERROR;
		}
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
	}

	return HAL_OK;
}

/* Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 */
HAL_StatusTypeDef EEPROM_PageErase (uint16_t page) {
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[PAGE_SIZE];
	memset(data,0xff,PAGE_SIZE);

	// write the data to the EEPROM
	if (HAL_I2C_Mem_Write(EEPROM_I2C, EEPROM_ADDR, MemAddress, 2, data, PAGE_SIZE, 1000) != HAL_OK) {
		return HAL_ERROR;
	}

	HAL_Delay (5);  // write cycle delay 

	return HAL_OK;
}
