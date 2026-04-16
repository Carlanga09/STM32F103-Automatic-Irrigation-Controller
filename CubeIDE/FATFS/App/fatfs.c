/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#include "fatfs.h"

uint8_t retUSER;    /* Return value for USER */
char USERPath[4];   /* USER logical drive path */
FATFS USERFatFS;    /* File system object for USER logical drive */
FIL USERFile;       /* File object for USER */

/* USER CODE BEGIN Variables */

// Use a fixed date (F103 RTCv1 doesn't have Date)
extern RTC_TimeTypeDef RTC_Time;
uint16_t year = 2025;  // must be >=1980
uint8_t  month = 10;
uint8_t  day = 18;
/* USER CODE END Variables */

void MX_FATFS_Init(void)
{
  /*## FatFS: Link the USER driver ###########################*/
  retUSER = FATFS_LinkDriver(&USER_Driver, USERPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
    return  ((DWORD)(year - 1980) << 25)
          | ((DWORD)month << 21)
          | ((DWORD)day << 16)
          | ((DWORD)RTC_Time.Hours << 11)
          | ((DWORD)RTC_Time.Minutes << 5)
          | ((DWORD)(RTC_Time.Seconds / 2));
  /* USER CODE END get_fattime */
}

/* USER CODE BEGIN Application */

/* USER CODE END Application */
