/*
 * W25Qxx.h
 *
 *  Created on: Nov 5, 2023
 *      Author: TueNV
 */

#ifndef INC_W25QXX_H_
#define INC_W25QXX_H_

extern SPI_HandleTypeDef hspi2;
#define W25Q_SPI hspi2
#define numBLOCK 32 // 32 number of total blocks for 16Mb flash, 32*16 sector, 32x16x16 pages and 32x16x16x256 bytes//
// 256 block = 256*16 sector ( 1 sector 4K) = 256*16*16 page ( 1 page 256 bytes)
void W25Q_Reset (void);
void W25Q_WaitForWriteEnd(void);
uint32_t W25Q_ReadID (void);
void W25Q_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData);
void W25Q_FastRead_address(uint32_t memAddr, uint32_t size, uint8_t *rData);
void W25Q_EraseChip(void);
void W25Q_Erase_Sector (uint16_t numsector);
void W25Q_Write_Clean(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);
void W25Q_Write(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data);

uint8_t W25Q_Read_Byte(uint32_t Addr);
void W25Q_Write_Byte(uint32_t Addr, uint8_t data);
void W25Q_Write_Nbytes(uint32_t Addr, uint8_t *data, uint32_t len);
void W25Q_Write_NUM(uint32_t page, uint16_t offset, float data);
float W25Q_Read_NUM(uint32_t page, uint16_t offset);
void W25Q_Write_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
void W25Q_Read_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data);
#endif /* INC_W25QXX_H_ */
