/*
 * W5500_SPI.c
 *
 *  Created on: Nov 4, 2022
 *      Author: Tueng
 */
#include "stm32f1xx_hal.h"
#include "stdio.h"

#include "../Ethernet/wizchip_conf.h"

extern SPI_HandleTypeDef hspi1;

void wizchip_select(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void wizchip_deselect(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t wizchip_read(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(&hspi1, &rbuf, 1, 0xFFFFFFFF);
//	HAL_SPI_Receive_DMA(&hspi1, &rbuf, 1);
	return rbuf;
}

void wizchip_write(uint8_t b) {
	HAL_SPI_Transmit(&hspi1, &b, 1, 0xFFFFFFFF);
//	HAL_SPI_Transmit_DMA(&hspi1, &b, 1);
}

void wizchip_readburst(uint8_t* pBuf, uint16_t len)
{
	HAL_SPI_Receive(&hspi1, pBuf, len, 0xFFFFFFFF);
//	HAL_SPI_Receive_DMA(&hspi1, pBuf, len);
}

void wizchip_writeburst(uint8_t* pBuf, uint16_t len)
{
	HAL_SPI_Transmit(&hspi1, pBuf, len, 0xFFFFFFFF);
//	HAL_SPI_Transmit_DMA(&hspi1, pBuf, len);
}

void W5500Init()
{
	uint8_t tmp = 0xFF;
	uint8_t memsize[2][8] = {{16,0,0,0,0,0,0,0},{16,0,0,0,0,0,0,0}};

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	while (tmp--);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
	reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
	reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);
	if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1)
	{
		while (1);
	}
}
