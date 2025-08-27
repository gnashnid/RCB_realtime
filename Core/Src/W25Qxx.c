/*
 * W25Qxx.c
 *
 *  Created on: Nov 5, 2023
 *      Author: TueNV
 */

#include "main.h"
#include "W25Qxx.h"

uint8_t tempBytes[4];
void csLOW(void)
{
	HAL_GPIO_WritePin(CS_W25_GPIO_Port, CS_W25_Pin, GPIO_PIN_RESET);
}
void csHIGH(void)
{
	HAL_GPIO_WritePin(CS_W25_GPIO_Port, CS_W25_Pin, GPIO_PIN_SET);
}
void W25Q_delay(uint32_t time)
{
	HAL_Delay(time);
}
void SPI_Write(uint8_t *data, uint16_t len)
{
	HAL_SPI_Transmit(&W25Q_SPI, data, len, 2000);
//	HAL_SPI_Transmit_DMA(&W25Q_SPI, data, len);
}
void SPI_Read(uint8_t *data, uint16_t len)
{
	HAL_SPI_Receive(&W25Q_SPI, data, len, 5000);
//	HAL_SPI_Receive_DMA(&W25Q_SPI, data, len);
}
//uint8_t	W25Q_Spi(uint8_t Data)
//{
//	uint8_t	ret;
//	HAL_SPI_TransmitReceive(&W25Q_SPI,&Data,&ret,1,100);
//	return ret;
//}
void W25Q_WaitForWriteEnd(void)
{
	uint8_t w25q_stt;
	uint8_t tData = 0x05;
	W25Q_delay(1);

	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, &tData, 1, 1000);
//	HAL_SPI_Transmit_DMA(&W25Q_SPI, &tData, 1);
	do
	{
		HAL_SPI_Receive(&W25Q_SPI, &w25q_stt, 1, 1000);
//		HAL_SPI_Receive_DMA(&W25Q_SPI, &w25q_stt, 1);
		W25Q_delay(1);
	}
	while ((w25q_stt & 0x01) == 0x01);
	csHIGH();
}
void W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66; //  enable reset
	tData[1] = 0x99; //reset
	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, tData, 2, 1000);
	csHIGH();
}
uint32_t W25Q_ReadID (void)
{
	uint8_t tData = 0x9F; // read JEDEC ID
	uint8_t rData[3];
	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, &tData, 1, 1000);;
//	HAL_SPI_Transmit_DMA(&W25Q_SPI, &tData, 1);

	HAL_SPI_Receive(&W25Q_SPI, rData, 3, 3000);
//	HAL_SPI_Receive_DMA(&W25Q_SPI, rData, 3);
	csHIGH();
	return ((rData[0]<<16)|(rData[1]<<8)|rData[2]);
}
void W25Q_Read(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[5];
	uint32_t memAddr = (startPage*256) + offset;
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x03;// enable read
		tData[1] = (memAddr>>16)&0xFF;// MSB addr
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr>>0)&0xFF;// LSB addr
	} else
	{
		tData[0] = 0x03;// enable read
		tData[1] = (memAddr>>24)&0xFF;// MSB addr
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;// LSB addr;
		tData[4] = (memAddr>>0)&0xFF;// LSB addr
	}
	csLOW();
	if (numBLOCK<1024)// size < 512mb
	{
		SPI_Write(tData, 4);
	} else
	{
		SPI_Write(tData, 5);
	}
	SPI_Read(rData, size);
	csHIGH();
}
void W25Q_FastRead(uint32_t startPage, uint8_t offset, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	uint32_t memAddr = (startPage*256) + offset;
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x0B;// enable fast read
		tData[1] = (memAddr>>16)&0xFF;// MSB addr
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr>>0)&0xFF;// LSB addr
		tData[4] = 0;
	} else
	{
		tData[0] = 0x0B;// enable fast read
		tData[1] = (memAddr>>24)&0xFF;// MSB addr
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;// LSB addr;
		tData[4] = (memAddr>>0)&0xFF;// LSB addr
		tData[5] = 0;
	}
	csLOW();
	if (numBLOCK<1024)// size < 512mb
	{
		SPI_Write(tData, 5);
	} else
	{
		SPI_Write(tData, 6);
	}
	SPI_Read(rData, size);
	csHIGH();
}

void W25Q_FastRead_address(uint32_t memAddr, uint32_t size, uint8_t *rData)
{
	uint8_t tData[6];
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x0B;// enable fast read
		tData[1] = (memAddr>>16)&0xFF;// MSB addr
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr>>0)&0xFF;// LSB addr
		tData[4] = 0;
	} else
	{
		tData[0] = 0x0B;// enable fast read
		tData[1] = (memAddr>>24)&0xFF;// MSB addr
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;// LSB addr;
		tData[4] = (memAddr>>0)&0xFF;// LSB addr
		tData[5] = 0;
	}
	csLOW();
	if (numBLOCK<1024)// size < 512mb
	{
		SPI_Write(tData, 5);
	} else
	{
		SPI_Write(tData, 6);
	}
	SPI_Read(rData, size);
	csHIGH();
}


void write_enable(void)
{
	uint8_t tData = 0x06; // enable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_delay(1);
}
void write_disable(void)
{
	uint8_t tData = 0x04; // disable write
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_delay(1);
}
uint32_t bytestowrite (uint32_t size, uint16_t offset)
{
	if ((size+offset)<256) return size;
	else return 256-offset;
}
uint32_t bytestomodify (uint32_t size, uint16_t offset)
{
	if ((size+offset)<4096) return size;
	else return 4096-offset;
}
void W25Q_EraseChip(void)
{
	uint8_t tData = 0xC7;
	write_enable();
	csLOW();
	SPI_Write(&tData, 1);
	csHIGH();
	W25Q_WaitForWriteEnd();
	W25Q_delay(5);
}
void W25Q_Erase_Sector (uint16_t numsector)
{
	uint8_t tData[6];
	uint32_t memAddr = numsector*16*256; // mỗi sector có 16 page * 256 byte
	W25Q_WaitForWriteEnd();
	write_enable();
	if (numBLOCK<512)
	{
		tData[0] = 0x20;//Erase sector
		tData[1] = (memAddr>>16)&0xFF;// MSB addr
		tData[2] = (memAddr>>8)&0xFF;
		tData[3] = (memAddr>>0)&0xFF;// LSB addr
		csLOW();
		SPI_Write(tData, 4);
		csHIGH();
	} else
	{

		tData[0] = 0x20;//Erase sector
		tData[1] = (memAddr>>24)&0xFF;// MSB addr
		tData[2] = (memAddr>>16)&0xFF;
		tData[3] = (memAddr>>8)&0xFF;// LSB addr;
		tData[4] = (memAddr>>0)&0xFF;// LSB addr
		csLOW();
		SPI_Write(tData, 5);
		csHIGH();
	}
	W25Q_WaitForWriteEnd();
	write_disable();
}
void W25Q_Write_Clean(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint8_t tData[266];
	uint32_t startPage = page;
	uint32_t endPage = startPage + ((size+offset-1)/256);
	uint32_t numPages = endPage - startPage +1;
	uint16_t startSector = startPage/16;
	uint16_t endSector = endPage/16;
	uint16_t numSectors = endSector -startSector + 1;
	for (uint16_t i=0; i<numSectors; i++)
	{
		W25Q_Erase_Sector(startSector++);
	}
	uint32_t dataPosition = 0;
	// write data
	for (uint32_t i=0; i<numPages; i++)
	{
		uint32_t memAddr = (startPage*256) + offset;
		uint16_t bytesremaining = bytestowrite(size, offset);
		uint32_t indx = 0;
		write_enable();
		if (numBLOCK<512)
		{
			tData[0] = 0x02;
			tData[1] = (memAddr>>16)&0xFF;// MSB addr
			tData[2] = (memAddr>>8)&0xFF;
			tData[3] = (memAddr>>0)&0xFF;// LSB addr
			indx = 4;
		} else
		{
			tData[0] = 0x02;
			tData[1] = (memAddr>>24)&0xFF;// MSB addr
			tData[2] = (memAddr>>16)&0xFF;
			tData[3] = (memAddr>>8)&0xFF;// LSB addr;
			tData[4] = (memAddr>>0)&0xFF;// LSB addr
			indx = 5;
		}
		uint16_t bytestosend = bytesremaining + indx;
		for (uint16_t i=0; i<bytesremaining; i++)
		{
			tData[indx++] = data[i+dataPosition];
		}
		if (bytestosend > 250)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytestosend-100);
			csHIGH();
		} else
		{
			csLOW();
			SPI_Write(tData, bytestosend);
			csHIGH();
		}
		startPage++;
		offset = 0;
		size = size-bytesremaining;
		dataPosition = dataPosition+bytesremaining;
//		W25Q_delay(5);
		W25Q_WaitForWriteEnd();
		write_disable();
	}
}
void W25Q_Write(uint32_t page, uint16_t offset, uint32_t size, uint8_t *data)
{
	uint16_t startSector = page/16;
	uint16_t endSector = (page + ((size+offset-1)/256))/16;
	uint16_t numSectors = endSector -startSector + 1;
	uint8_t previousData[4096];
	uint32_t sectorOffset = ((page%16)*256)+offset;
	uint32_t dataindx = 0;
	for (uint16_t i=0; i<numSectors; i++)
	{
		uint32_t startPage = startSector*16;
		W25Q_FastRead(startPage, 0, 4096, previousData);
		uint16_t bytesRemaining = bytestomodify(size, sectorOffset);
		for (uint16_t i=0; i<bytesRemaining; i++)
		{
			previousData[i+sectorOffset] = data[i+dataindx];
		}
		W25Q_Write_Clean(startPage, 0, 4096, previousData);
		startSector++;
		sectorOffset = 0;
		dataindx = dataindx + bytesRemaining;
		size = size - bytesRemaining;
	}
}
uint8_t W25Q_Read_Byte(uint32_t Addr)
{
	uint8_t tData[5];
	uint8_t rData;
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x03;// enable read
		tData[1] = (Addr>>16)&0xFF;// MSB addr
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr>>0)&0xFF;// LSB addr
	} else
	{
		tData[0] = 0x03;// enable read
		tData[1] = (Addr>>24)&0xFF;// MSB addr
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;// LSB addr;
		tData[4] = (Addr>>0)&0xFF;// LSB addr
	}
	csLOW();
	if (numBLOCK<1024)// size < 512mb
	{
		SPI_Write(tData, 4);
	} else
	{
		SPI_Write(tData, 5);
	}
	SPI_Read(&rData, 1);
	csHIGH();
	return rData;
}
void W25Q_Write_Byte(uint32_t Addr, uint8_t data)
{
	uint8_t tData[6];
	uint8_t indx;
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x02;// enable read
		tData[1] = (Addr>>16)&0xFF;// MSB addr
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr>>0)&0xFF;// LSB addr
		tData[4] = data;
		indx = 5;
	} else
	{
		tData[0] = 0x02;// enable read
		tData[1] = (Addr>>24)&0xFF;// MSB addr
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;// LSB addr;
		tData[4] = (Addr>>0)&0xFF;// LSB addr
		tData[5] = data;
		indx = 6;
	}
	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		write_enable();
		csLOW();
		SPI_Write(tData, indx);
		csHIGH();
//		W25Q_delay(5);
		W25Q_WaitForWriteEnd();
		write_disable();
	}
}
void W25Q_Write_Nbytes(uint32_t Addr, uint8_t *data, uint32_t len)
{
	uint8_t tData[266];
	uint8_t indx;
	if (numBLOCK<1024)// size < 512mb
	{
		tData[0] = 0x02;// enable read
		tData[1] = (Addr>>16)&0xFF;// MSB addr
		tData[2] = (Addr>>8)&0xFF;
		tData[3] = (Addr>>0)&0xFF;// LSB addr
		indx = 4;
	} else
	{
		tData[0] = 0x02;// enable read
		tData[1] = (Addr>>24)&0xFF;// MSB addr
		tData[2] = (Addr>>16)&0xFF;
		tData[3] = (Addr>>8)&0xFF;// LSB addr;
		tData[4] = (Addr>>0)&0xFF;// LSB addr
		indx = 5;
	}
	uint16_t bytestosend = len + indx;
	for (uint16_t i=0; i<len; i++)
	{
		tData[indx++] = data[i];
	}
	if (W25Q_Read_Byte(Addr) == 0xFF)
	{
		write_enable();
		if (bytestosend > 250)
		{
			csLOW();
			SPI_Write(tData, 100);
			SPI_Write(tData+100, bytestosend-100);
			csHIGH();
		} else
		{
			csLOW();
			SPI_Write(tData, bytestosend);
			csHIGH();
		}
//		W25Q_delay(5);
		W25Q_WaitForWriteEnd();
		write_disable();
	}
}
void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}
void W25Q_Write_NUM(uint32_t page, uint16_t offset, float data)
{
	float2Bytes(tempBytes, data);
//	uint32_t Addr = (page*256) + offset;
//	for(uint8_t i=0; i<4; i++)
//	{
//		W25Q_Write_Byte(i+Addr, tempBytes[i]);
//	}
	W25Q_Write(page, offset, 4, tempBytes);
}
float W25Q_Read_NUM(uint32_t page, uint16_t offset)
{
	uint8_t rData[4];
	W25Q_Read(page, offset, 4, rData);
	return (Bytes2float(rData));
}
void W25Q_Write_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx;
	for (uint32_t i=0; i<size; i++)
	{
		data8[indx++] = data[i]&0xFF;
		data8[indx++] = (data[i]>>8)&0xFF;
		data8[indx++] = (data[i]>>16)&0xFF;
		data8[indx++] = (data[i]>>24)&0xFF;
	}
	W25Q_Write(page, offset, indx, data8);
}
void W25Q_Read_32B(uint32_t page, uint16_t offset, uint32_t size, uint32_t *data)
{
	uint8_t data8[size*4];
	uint32_t indx = 0;
	W25Q_FastRead(page, offset, size*4, data8);
	for (uint32_t i=0; i<size; i++)
	{
		data[i] = (data8[indx]) | (data8[indx+1]<<8) | (data8[indx+2]<<16) | (data8[indx+3]<<24);
		indx += 4;
	}
}
