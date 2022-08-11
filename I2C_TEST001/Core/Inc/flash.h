/*
 * flash.h
 *
 *  Created on: 2022. 8. 11.
 *      Author: DELL
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


// value form for using the flash example
//#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_3   /* Start @ of user Flash area */
//#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_23  +  GetSectorSize(ADDR_FLASH_SECTOR_23) -1 /* End @ of user Flash area : sector start address + sector size -1 */
//#define DATA_32                 ((uint32_t)0x12345678)
/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_12     ((uint32_t)0x08100000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_13     ((uint32_t)0x08104000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_14     ((uint32_t)0x08108000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_15     ((uint32_t)0x0810C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_16     ((uint32_t)0x08110000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_17     ((uint32_t)0x08120000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_18     ((uint32_t)0x08140000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_19     ((uint32_t)0x08160000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_20     ((uint32_t)0x08180000) /* Base @ of Sector 8, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_21     ((uint32_t)0x081A0000) /* Base @ of Sector 9, 128 Kbytes  */
#define ADDR_FLASH_SECTOR_22     ((uint32_t)0x081C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_23     ((uint32_t)0x081E0000) /* Base @ of Sector 11, 128 Kbytes */
/* USER CODE END EM */

uint32_t GetSector(uint32_t Address);
uint32_t GetSectorSize(uint32_t Sector);
void FlashWritingOne (uint32_t FLASH_USER_ADDR, uint32_t DATA_32) ;
void FlashWritingRange (uint32_t FLASH_USER_START_ADDR, uint32_t FLASH_USER_END_ADDR, uint32_t DATA_32) ;

//uint32_t FirstSector = 0, NbOfSectors = 0;
//uint32_t Address = 0, SECTORError = 0;
//__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;
///*Variable used for Erase procedure*/
//static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE BEGIN 2 */


///* Unlock the Flash to enable the flash control register access *************/
//HAL_FLASH_Unlock();
//
///* Erase the user Flash area
//  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
///* Get the 1st sector to erase */
//FirstSector = GetSector(FLASH_USER_START_ADDR);
///* Get the number of sector to erase from 1st sector*/
//NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;
///* Fill EraseInit structure*/
//EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
//EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
//EraseInitStruct.Sector        = FirstSector;
//EraseInitStruct.NbSectors     = NbOfSectors;
//
///* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//   you have to make sure that these data are rewritten before they are accessed during code
//   execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//   DCRST and ICRST bits in the FLASH_CR register. */
//if(HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
//{
//}
//
///* Program the user Flash area word by word
//  (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//
//Address = FLASH_USER_START_ADDR;
//
//while(Address < FLASH_USER_END_ADDR)
//{
//  if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) == HAL_OK)
//  {
//    Address = Address + 4;
//  }
//
//}
//
///* Lock the Flash to disable the flash control register access (recommended
//   to protect the FLASH memory against possible unwanted operation) *********/
//HAL_FLASH_Lock();


/* USER CODE END 2 */


#endif /* INC_FLASH_H_ */
