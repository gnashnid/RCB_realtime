/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wizchip_conf.h"
#include "socket.h"
#include "W5500_SPI.h"
#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "stdbool.h"
#include "W25Qxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

FLASH_EraseInitTypeDef EraseInit;
#define SOCK_NUM 0
#define Timeout_online 1000*10
#define Timeout_heartbit 10000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader_MC2B;

uint8_t uart_data[50], buf[1024], server_ip[4], time_delay, lock_default[8], ip1, ip2, ip3, ip4,
		ip_server1, ip_server2, ip_server3, ip_server4, data_info[30], elevator_mode, uart_bypass = 3,
		data_MC2B[8], permission_REBcard[8] = {0xFF}, c[2]={0x48, 0}, keepalive, Ethernet_received_data[9],
		Ethernet_setting_data[25], Ethernet_read_and_reset_data, counter_start, RxData[8], send_uart;
uint16_t port_server, port_client;
uint32_t SectorError, wcode, TxMailbox, x_timer[64], number_card, connected, time_break, lenData, number_card_old,
		 lock_default_1, lock_default_2, time_check, time_auto_reconnect, heardbit_REB, timeSendDataEth,
		 RST_timer, RST_timer_now, RST_timer_last, count_RST, counter_reset, time_now;
bool bypass_from_REB, bypass_from_Eth, x[64], Ethernet_connected, isSendDataEth, write_mode_somecard,
	 write_mode, write_done, new_card, send_card_to_pc, send_card_done,
	 linkport, REB_connected = true, Ethernet_received, Ethernet_setting, add_card_uart,
	 Ethernet_read_and_reset, RST_set, unlock_fire, send_uart_to_REB, send_uart_to_PC;


// Định nghĩa năm epoch (ví dụ: 1/1/2000 00:00:00)
#define EPOCH_YEAR 2025
// Số ngày trong các tháng (không tính năm nhuận)
static const int days_in_month[] = {
    31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

//volatile bool ip_assigned = false;
//
//
typedef struct {
	uint32_t STT;
	uint32_t cardID;
	uint8_t permis[8];
	uint32_t time_up;
	uint32_t time_dow;
} user_info_t;
#pragma pack()

user_info_t user, user_before, compare_user, write_user, send_user;

extern uint8_t g_uart_request_exit;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void send_u8_eth (char *CMD, uint8_t data);
void sendData_eth (char *CMD, uint32_t data);
void sendData_eth_info (char *CMD, uint8_t *data);
void sendData_uart (char *CMD, uint8_t data);
void sendData_eth_CardID (char *CMD, user_info_t user);
void save_data();
void sendString (char *CMD, char *data);
void sendString_uart (char *CMD);
void sendString_info_uart (uint8_t *CMD);
//void write_sector (uint32_t Start_STT);
user_info_t binary_search(uint32_t Number_card, uint32_t code);
void new_card_update(uint8_t *data, bool *input, uint32_t *input_timer);
void calculate_data_can(bool *input, uint32_t *input_timer, uint8_t *data_can, uint8_t *unlock_fl, uint8_t delay_time, uint8_t *counter_start);
uint8_t reconect_eth(uint8_t sn);
void Set_speed_can(uint8_t speed);
void shift_left_1bit( uint8_t input[8], uint8_t output[8]);
// Hàm kiểm tra năm nhuận
static uint8_t is_leap_year(uint16_t year);
// Hàm tùy chỉnh tương tự mktime
uint32_t mktime(uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == huart1.Instance)
	{
		heardbit_REB = HAL_GetTick(); // Heard bit from REB
		switch (uart_data[0])
		{
		case 0x44: //Data: D
			wcode = uart_data[1]<<24|uart_data[2]<<16|uart_data[3]<<8|uart_data[4];
			if (!write_mode && !write_mode_somecard)
			{
				sendData_eth("D", wcode);
				isSendDataEth = true;
				timeSendDataEth = HAL_GetTick();
			}
			break;
		case 0x43://Comand: C
//			send_uart_to_REB = true;
			if (send_uart_to_REB)
			{
				send_uart_to_REB = false;
				sendData_uart("I", send_uart);
			} else
			{
				sendData_uart("I", uart_bypass);
			}
//			time_send_uart = HAL_GetTick();
			bypass_from_REB = uart_data[4];
			break;
		case 0x53://setting: S
			Ethernet_setting = true;
			send_uart_to_PC = true;
			for (uint8_t i =0; i<23; i++)
			{
				Ethernet_setting_data[i] = uart_data[i+1];
			}
			g_uart_request_exit = 1;
			break;
		case 0x52://Read R
			if (uart_data[1] == 0x01)
			{
				  Ethernet_read_and_reset = true;
				  Ethernet_read_and_reset_data = uart_data[1];
				  send_uart_to_PC = true;
			}
			g_uart_request_exit = 1;
			break;
		case 0x57: // W
			switch (uart_data[1])
			{
			case 0x41:// E: errase
				write_mode_somecard = true;
				time_break = HAL_GetTick();
				send_uart_to_PC = true;
				add_card_uart = true;
				g_uart_request_exit = 1;
				break;
			case 0x45:// E: errase
				write_mode = true;
				time_break = HAL_GetTick();
				send_uart_to_PC = true;
				add_card_uart = true;
				g_uart_request_exit = 1;
				break;
			case 0x44:
				  write_user.STT++;
				  write_user.cardID =  uart_data[2]<<24|uart_data[3]<<16|uart_data[4]<<8|uart_data[5];
				  write_user.permis[0] = uart_data[6];
				  write_user.permis[1] = uart_data[7];
				  write_user.permis[2] = uart_data[8];
				  write_user.permis[3] = uart_data[9];
				  write_user.permis[4] = uart_data[10];
				  write_user.permis[5] = uart_data[11];
				  write_user.permis[6] = uart_data[12];
				  write_user.permis[7] = uart_data[13];
				  new_card = true;
				  send_uart_to_PC = true;
				break;
			case 0x43:
				  write_done = true;
				  send_uart_to_PC = true;
				break;
			}
			break;
		}
	}
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_data, 50);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == RST_Pin)
	{
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin) == 0)
		{
			RST_set = true;
			RST_timer = HAL_GetTick();
			count_RST = 2000;
			while (!HAL_GPIO_ReadPin(RST_GPIO_Port, RST_Pin))
			{
				RST_timer_last = HAL_GetTick() - RST_timer;
//				if ((RST_timer_last > 5000) && (RST_timer_last < 10000))
//				{
//					count_RST = 500;
//				} else
				if (RST_timer_last > 10000)
				{
					count_RST = 100;
				}
				if (HAL_GetTick() - RST_timer_now > count_RST)
				{
					RST_timer_now = HAL_GetTick();
					HAL_GPIO_TogglePin(LED_STT_GPIO_Port, LED_STT_Pin);
				}
				HAL_Delay(10);
			}
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim2.Instance)
	{
		// calculator data
		if (counter_start < time_delay)
		{
			counter_start++;
		}
		if (write_mode || write_mode_somecard || unlock_fire || bypass_from_Eth || bypass_from_REB || !REB_connected || !HAL_GPIO_ReadPin(BYPASS_GPIO_Port, BYPASS_Pin))
		{
			data_MC2B[0] = 1;
			data_MC2B[1] = 0;
			data_MC2B[2] = 0;
			data_MC2B[3] = 0;
			data_MC2B[4] = 0;
			data_MC2B[5] = 0;
			data_MC2B[6] = 0;
			data_MC2B[7] = 0;
			HAL_GPIO_WritePin(LED_BP_GPIO_Port, LED_BP_Pin, 1);
//			sendData_uart("I", 2);
			uart_bypass = 2;
//			send_uart_to_REB = true;
		} else
		{
			calculate_data_can(x, x_timer, data_MC2B, lock_default, time_delay, &counter_start);
			HAL_GPIO_WritePin(LED_BP_GPIO_Port, LED_BP_Pin, 0);
			uart_bypass = 3;
	//		send_uart_to_REB = true;
		}
		HAL_CAN_AddTxMessage(&hcan, &TxHeader_MC2B, data_MC2B, &TxMailbox);
//		sendData_uart("I", 3);
	}
	if (htim->Instance == htim3.Instance)
	{
		HAL_IWDG_Refresh(&hiwdg);
		if (!RST_set)
		{
			HAL_GPIO_TogglePin(LED_STT_GPIO_Port, LED_STT_Pin);
		}
	}
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
	    Error_Handler();
	}
	if (RxHeader.StdId == 0x740 && RxData[2] == 0x48)
	{
		if ((RxData[4] & 0x40) == 0x40)
		{
			unlock_fire = true;
		} else
		{
			unlock_fire = false;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  __HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_data, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  TxHeader_MC2B.DLC = 8;
  TxHeader_MC2B.ExtId = 0x47FF;
  TxHeader_MC2B.IDE = CAN_ID_EXT;
  TxHeader_MC2B.RTR = CAN_RTR_DATA;
  TxHeader_MC2B.StdId = 0;
  TxHeader_MC2B.TransmitGlobalTime = DISABLE;

  EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInit.Banks = FLASH_BANK_1;
  EraseInit.PageAddress =  0x0800FC00;
  EraseInit.NbPages = 1;
  ip1 = *(uint32_t *)(0x0800FC00);
  ip2 = *(uint32_t *)(0x0800FC04);
  ip3 = *(uint32_t *)(0x0800FC08);
  ip4 = *(uint32_t *)(0x0800FC0C);
  port_client = *(uint32_t *)(0x0800FC10);
  ip_server1 = *(uint32_t *)(0x0800FC14);
  ip_server2 = *(uint32_t *)(0x0800FC18);
  ip_server3 = *(uint32_t *)(0x0800FC1C);
  ip_server4 = *(uint32_t *)(0x0800FC20);
  port_server = *(uint32_t *)(0x0800FC24);
  time_delay = *(uint32_t *)(0x0800FC28);
  elevator_mode = *(uint32_t *)(0x0800FC2C);
  lock_default_1 = *(uint32_t *)(0x0800FC30);
  lock_default_2 = *(uint32_t *)(0x0800FC34);
  if (ip1 == 0xFF) ip1 = 192;
  if (ip2 == 0xFF) ip2 = 168;
  if (ip3 == 0xFF) ip3 = 0;
  if (ip4 == 0xFF) ip4 = 72;
  if (port_client == 0xFFFF) port_client = 0;
  if (ip_server1 == 0xFF) ip_server1 = 192;
  if (ip_server2 == 0xFF) ip_server2 = 168;
  if (ip_server3 == 0xFF) ip_server3 = 0;
  if (ip_server4 == 0xFF) ip_server4 = 2;
  if (port_server == 0xFFFF) port_server = 6000;
  if (time_delay == 0xFF) time_delay = 1;
  if (elevator_mode == 0xFF) elevator_mode = 0;
//  if (lock_default_1 == 0xFFFFFFFF) lock_default_1 = 0;
//  if (lock_default_2 == 0xFFFFFFFF) lock_default_2 = 0;
  lock_default[0] = lock_default_1 & 0xFF;
  lock_default[1] = (lock_default_1>>8) & 0xFF;
  lock_default[2] = (lock_default_1>>16) & 0xFF;
  lock_default[3] = (lock_default_1>>24) & 0xFF;
  lock_default[4] = lock_default_2 & 0xFF;
  lock_default[5] = (lock_default_2>>8) & 0xFF;
  lock_default[6] = (lock_default_2>>16) & 0xFF;
  lock_default[7] = (lock_default_2>>24) & 0xFF;
  server_ip[0] = ip_server1;
  server_ip[1] = ip_server2;
  server_ip[2] = ip_server3;
  server_ip[3] = ip_server4;

  W25Q_Reset();
//  W25Q_EraseChip();
  while(user.STT!=0xFFFFFFFF)
  {
	  W25Q_FastRead_address(number_card*24, sizeof(user_info_t), (uint8_t *)&user);
	  if (user.STT -1 == number_card)
	  {
		  number_card = user.STT;
	  }
  }
  while(user_before.STT!=0xFFFFFFFF)
  {
	  W25Q_FastRead_address(number_card_old*24, sizeof(user_info_t), (uint8_t *)&user_before);
	  if (user_before.STT -1 == number_card_old)
	  {
		  number_card_old = user_before.STT;
	  }
  }
  if (number_card_old > 0)
  {
	  for (uint32_t i=0; i<((number_card_old*24/(16*256))+2); i++)
	  {
		  W25Q_Erase_Sector(i+16*33);
	  }
  }
  wiz_NetInfo gWIZNETINFO = {
  		  .mac = {0x00, 0x1c, 0x16, ip4, 0xFF-ip4, ip4*ip4},
  		  .ip = {ip1,ip2,ip3,ip4},
  		  .sn = {255, 255, 255, 0},
  		  .gw = {192, 168, 0, 1},
  		  .dns = {168, 126, 63, 1},
  		  .dhcp = NETINFO_STATIC};
  server_ip[0] = ip_server1;
  server_ip[1] = ip_server2;
  server_ip[2] = ip_server3;
  server_ip[3] = ip_server4;
  gWIZNETINFO.mac[3] = ((HAL_GetUIDw0()>>24)&0xFF) + ((HAL_GetUIDw2()>>16)&0xFF) + ((HAL_GetUIDw1()>>8)&0xFF) + (HAL_GetUIDw0()&0xFF);
  gWIZNETINFO.mac[4] = ((HAL_GetUIDw1()>>24)&0xFF) + ((HAL_GetUIDw0()>>16)&0xFF) + ((HAL_GetUIDw2()>>8)&0xFF) + (HAL_GetUIDw2()&0xFF);
  gWIZNETINFO.mac[5] = ((HAL_GetUIDw2()>>24)&0xFF) + ((HAL_GetUIDw1()>>16)&0xFF) + ((HAL_GetUIDw0()>>8)&0xFF) + (HAL_GetUIDw1()&0xFF);
  W5500Init();
  HAL_Delay(2000);
  wizchip_setnetinfo(&gWIZNETINFO);
  socket(SOCK_NUM, Sn_MR_TCP, port_client, SF_TCP_NODELAY);
  connect(SOCK_NUM, server_ip, port_server);
  if (getSn_SR(SOCK_NUM) == SOCK_ESTABLISHED)
  {
	  time_check = HAL_GetTick();
	  time_auto_reconnect = HAL_GetTick();
	  HAL_GPIO_WritePin(LED_STT_ETH_GPIO_Port, LED_STT_ETH_Pin, GPIO_PIN_SET);
  }
  Set_speed_can(elevator_mode);
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  lenData = getSn_RX_RSR(SOCK_NUM);
	  if (getSn_RX_RSR(SOCK_NUM) == lenData)
	  {
		  if (lenData > 1024)
		  {
			  HAL_NVIC_SystemReset();
		  }
		  if (lenData > 0)
		  {
			  recv(SOCK_NUM, buf, lenData);
			  switch (buf[0])
			  {
			  case 0x48:
				  connected = HAL_GetTick();
				  time_now = mktime((buf[3]<<8) & buf[4], buf[2], buf[1], buf[5], buf[6]);
				  break;
			  case 0x44://data: D
				  Ethernet_received = true;
				  for (uint8_t i =0; i<9; i++)
				  {
					  Ethernet_received_data[i] = buf[i+1];
				  }
				  break;
			  case 0x53://setting: S
				  Ethernet_setting = true;
				  for (uint8_t i =0; i<23; i++)
				  {
					  Ethernet_setting_data[i] = buf[i+1];
				  }
				  break;
			  case 0x52:// reset board R: 0; read data: 1
				  if (buf[1] == 2)
				  {
					  send_card_to_pc = true;
					  time_break = HAL_GetTick();
				  } else if (buf[1] == 3)
				  {
					  send_card_done = true;
					  time_break = HAL_GetTick();
				  } else
				  {
					  Ethernet_read_and_reset = true;
					  Ethernet_read_and_reset_data = buf[1];
				  }
				  break;
			  case 0x57: // W
				  switch (buf[1])
				  {
				  case 0x45:// E: errase
					  write_mode = true;
					  time_break = HAL_GetTick();
					  break;
				  case 0x41:// A: add card
					  write_mode_somecard = true;
					  time_break = HAL_GetTick();
					  break;
				  }
				  break;
			  }
		  }
	  } else
	  {
		  continue;
	  }

	  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
	  {
		  if (HAL_CAN_AbortTxRequest(&hcan, TxMailbox) != HAL_OK)
		  {
			HAL_NVIC_SystemReset();
		  }
	  }

	  // check connection Ethernet
	  if (keepalive == SOCK_ESTABLISHED && (abs(HAL_GetTick() - connected) < Timeout_heartbit))
	  {
		  Ethernet_connected = true;
	  } else
	  {
		  Ethernet_connected = false;// false
	  }
	  // check connection REB
	  if (abs(HAL_GetTick() - heardbit_REB) > Timeout_heartbit)
	  {
		  REB_connected = false;// false
	  } else
	  {
		  REB_connected = true;
	  }
	  //// received data from Ethernet
	  if (Ethernet_received)// 0x44
	  {
		  Ethernet_received = false;
		  switch (Ethernet_received_data[0])
		  {
		  case 0:
			  send_uart = 0;
			  send_uart_to_REB = true;
			  isSendDataEth = false;
			  break;
		  case 1: // data ok
			  new_card_update(&Ethernet_received_data[1], x, x_timer);
			  send_uart = 1;
			  send_uart_to_REB = true;
			  isSendDataEth = false;
			  break;
		  }
	  }

	  // Nếu không nhận được Data từ server sau khi gửi mã thẻ, thì xử lý offline

	  if (((isSendDataEth == true) && !Ethernet_connected) || ((isSendDataEth == true) && ((HAL_GetTick() - timeSendDataEth) > 500)))
	  {
		  isSendDataEth = false;
		  compare_user = binary_search(number_card, wcode);
		  if (compare_user.STT)
		  {
			  if ((compare_user.time_up < time_now) && (compare_user.time_dow > time_now))
			  {
				  new_card_update(compare_user.permis, x, x_timer);
				  send_uart = 1;
				  send_uart_to_REB = true;
			  }
		  } else
		  {
			  send_uart = 0;
			  send_uart_to_REB = true;
		  }
	  }
	  //// setting from PC Ethernet
	  if (Ethernet_setting) // 0x53
	  {
		  Ethernet_setting = false;
		  switch (Ethernet_setting_data[0])
		  {
		  case 0:// setting board
			  ip1 = Ethernet_setting_data[1];
			  ip2 = Ethernet_setting_data[2];
			  ip3 = Ethernet_setting_data[3];
			  ip4 = Ethernet_setting_data[4];
			  port_client = Ethernet_setting_data[5]<<8|Ethernet_setting_data[6];
			  ip_server1 = Ethernet_setting_data[7];
			  ip_server2 = Ethernet_setting_data[8];
			  ip_server3 = Ethernet_setting_data[9];
			  ip_server4 = Ethernet_setting_data[10];
			  port_server = Ethernet_setting_data[11]<<8|Ethernet_setting_data[12];
			  time_delay = Ethernet_setting_data[13];
			  elevator_mode = Ethernet_setting_data[14];
			  lock_default_1 = Ethernet_setting_data[15]|(Ethernet_setting_data[16]<<8)|(Ethernet_setting_data[17]<<16)|(Ethernet_setting_data[18]<<24);
			  lock_default_2 = Ethernet_setting_data[19]|(Ethernet_setting_data[20]<<8)|(Ethernet_setting_data[21]<<16)|(Ethernet_setting_data[22]<<24);
			  save_data();
			  if (send_uart_to_PC)
			  {
				  sendString_uart("SOK");
			  } else
			  {
				  sendString("S", "OK");
			  }
			  HAL_NVIC_SystemReset();
			  break;
		  case 1:// bypass mode
			  bypass_from_Eth = Ethernet_setting_data[1];
			  break;
		  }
	  }
	  /// reset or conmand read data from PC
	  if (Ethernet_read_and_reset)// 0x52
	  {
		  Ethernet_read_and_reset = false;
		  switch (Ethernet_read_and_reset_data)
		  {
		  case 0:
			  HAL_NVIC_SystemReset();
			  break;
		  case 1:
			  data_info[0] = ip1;
			  data_info[1] = ip2;
			  data_info[2] = ip3;
			  data_info[3] = ip4;
			  data_info[4] = port_client>>8;
			  data_info[5] = port_client & 0xFF;
			  data_info[6] = ip_server1;
			  data_info[7] = ip_server2;
			  data_info[8] = ip_server3;
			  data_info[9] = ip_server4;
			  data_info[10] = port_server>>8;
			  data_info[11] = port_server & 0xFF;
			  data_info[12] = time_delay;
			  data_info[13] = elevator_mode;
			  data_info[14] = number_card>>24&0xFF;
			  data_info[15] = number_card>>16&0xFF;
			  data_info[16] = number_card>>8&0xFF;
			  data_info[17] = number_card&0xFF;
			  data_info[18] = lock_default[0];
			  data_info[19] = lock_default[1];
			  data_info[20] = lock_default[2];
			  data_info[21] = lock_default[3];
			  data_info[22] = lock_default[4];
			  data_info[23] = lock_default[5];
			  data_info[24] = lock_default[6];
			  data_info[25] = lock_default[7];
			  if (send_uart_to_PC)
			  {
				  sendString_info_uart(data_info);
				  send_uart_to_PC = false;
			  } else
			  {
				  sendData_eth_info("R", data_info);
			  }
			  break;
		  }
	  }
	  //// send data to PC, to keep alive connection
	  if (!add_card_uart && (abs(HAL_GetTick() - time_check) > 5000))
	  {
		  time_check = HAL_GetTick();
		  keepalive = reconect_eth(SOCK_NUM);
		  if (keepalive == SOCK_ESTABLISHED)
		  {
			  if (unlock_fire)
			  {
				  c[1] = 0x46;//F
			  } else if (bypass_from_Eth)
			  {
				  c[1] = 0x50;//P
			  } else if (bypass_from_REB)
			  {
				  c[1] = 0x45;//E
			  } else if (!REB_connected)
			  {
				  c[1] = 0x44;//D
			  } else if (!HAL_GPIO_ReadPin(BYPASS_GPIO_Port, BYPASS_Pin))
			  {
				  c[1] = 0x43;//C
			  } else if (!Ethernet_connected)
			  {
				  c[1] = 0x4F;//o
			  } else
			  {
				  c[1] = 0x52;//r
			  }
			  send(SOCK_NUM, c, 2);
		  }
	  }

	  while (send_card_to_pc)
	  {
		  if (number_card == 0)
		  {
//			  sendString("R", "EMPTY");
			  send_u8_eth("X", 0);
			  send_card_to_pc = false;
		  } else
		  {
			  for (uint32_t i=0; i< number_card; i++)
			  {
				  W25Q_FastRead_address(i*24, sizeof(user_info_t), (uint8_t *)&send_user);
				  sendData_eth_CardID ("X", send_user);
				  HAL_Delay(100);
			  }
//					  sendString("R", "CMPLT");
			  send_u8_eth("X", 1);
			  send_card_to_pc = false;
		  }
	  }
	  while (write_mode)
	  {
		  uint32_t totalCard = 0;
		  if (send_uart_to_PC)
		  {
			  send_uart_to_PC = false;
			  sendString_uart("WEOK");
		  } else
		  {
			  sendString("W", "EOK");
		  }
		  time_break = HAL_GetTick();
		  bypass_from_Eth = true;
		  while (1)
		  {
			  lenData = getSn_RX_RSR(SOCK_NUM);
			  if (getSn_RX_RSR(SOCK_NUM) == lenData)
			  {
				  if (lenData > 1024)
				  {
					  HAL_NVIC_SystemReset();
				  }
				  if (!add_card_uart && (lenData > 0))
				  {
					  recv(SOCK_NUM, buf, lenData);
					  if (buf[0] == 0x57)
					  {
						  if (buf[1] == 0x44)
						  {
							  totalCard++;
							  write_user.STT = totalCard;
							  write_user.cardID =  buf[2]<<24|buf[3]<<16|buf[4]<<8|buf[5];
							  write_user.permis[0] = buf[6];
							  write_user.permis[1] = buf[7];
							  write_user.permis[2] = buf[8];
							  write_user.permis[3] = buf[9];
							  write_user.permis[4] = buf[10];
							  write_user.permis[5] = buf[11];
							  write_user.permis[6] = buf[12];
							  write_user.permis[7] = buf[13];
							  write_user.time_up = mktime((buf[16]<<8) & buf[17], buf[15], buf[14], buf[18], buf[19]);
							  write_user.time_dow = mktime((buf[22]<<8) & buf[23], buf[21], buf[20], buf[24], buf[25]);

							  W25Q_Write_Nbytes((totalCard-1)*24 + 0x210000, (uint8_t *)&write_user, sizeof(user_info_t));
							  sendString("W", "DOK");
							  time_break = HAL_GetTick();
						  } else if (buf[1] == 0x43)
						  {
							  write_done = true;
						  }
					  }
				  }
			  } else
			  {
				  continue;
			  }
			  if (new_card)
			  {
				  totalCard++;
				  W25Q_Write_Nbytes((totalCard-1)*24 + 0x210000, (uint8_t *)&write_user, sizeof(user_info_t));
				  new_card = false;
				  sendString_uart("WDOK");
				  time_break = HAL_GetTick();
			  }
			  while (write_done)
			  {
				  for (uint8_t i=0; i<(number_card)*24/(16*256)+1; i++)
				  {
					  W25Q_Erase_Sector(i);
				  }
				  for (uint32_t i = 0; i<totalCard; i++)
				  {
					  W25Q_FastRead_address(i*24+ 0x210000, sizeof(user_info_t), (uint8_t *)&send_user);
					  W25Q_Write_Nbytes(i*24, (uint8_t *)&send_user, sizeof(user_info_t));
				  }
				  for (uint32_t i=0; i<((totalCard*24/(16*256))+2); i++)
				  {
					  W25Q_Erase_Sector(i+16*33);
				  }
				  if (send_uart_to_PC)
				  {
					  send_uart_to_PC = false;
					  sendString_uart("WCOK");
				  } else
				  {
					  sendString("W", "COK");
				  }
				  HAL_NVIC_SystemReset();
			  }

			  if ((abs(HAL_GetTick() - time_break) > Timeout_online))
			  {
				  HAL_NVIC_SystemReset();
			  }
		  }
	  }
	  while (write_mode_somecard)
	  {
		  uint32_t add_card = 0;
		  if (send_uart_to_PC)
		  {
			  send_uart_to_PC = false;
			  sendString_uart("WAOK");
		  } else
		  {
			  sendString("W", "AOK");
			  HAL_Delay(50);
			  sendString("W", "AOK");
			  HAL_Delay(50);
			  sendString("W", "AOK");
		  }
		  time_break = HAL_GetTick();
		  bypass_from_Eth = true;
		  while (1)
		  {
			  lenData = getSn_RX_RSR(SOCK_NUM);
			  if (getSn_RX_RSR(SOCK_NUM) == lenData)
			  {
				  if (lenData > 1024)
				  {
					  HAL_NVIC_SystemReset();
				  }
				  if (!add_card_uart && (lenData > 0))
				  {
					  recv(SOCK_NUM, buf, lenData);
					  if (buf[0] == 0x57)
					  {
						  if (buf[1] == 0x44)
						  {
							  add_card++;
							  write_user.STT++;
							  write_user.cardID =  buf[2]<<24|buf[3]<<16|buf[4]<<8|buf[5];
							  write_user.permis[0] = buf[6];
							  write_user.permis[1] = buf[7];
							  write_user.permis[2] = buf[8];
							  write_user.permis[3] = buf[9];
							  write_user.permis[4] = buf[10];
							  write_user.permis[5] = buf[11];
							  write_user.permis[6] = buf[12];
							  write_user.permis[7] = buf[13];
							  write_user.time_up = mktime((buf[16]<<8) & buf[17], buf[15], buf[14], buf[18], buf[19]);
							  write_user.time_dow = mktime((buf[22]<<8) & buf[23], buf[21], buf[20], buf[24], buf[25]);

							  W25Q_Write_Nbytes((add_card-1)*24 + 0x210000, (uint8_t *)&write_user, sizeof(user_info_t));
							  sendString("W", "DOK");
							  time_break = HAL_GetTick();
						  } else if (buf[1] == 0x43)
						  {
							  write_done = true;
						  }
					  }
				  }
			  } else
			  {
				  continue;
			  }
			  if (new_card)
			  {
				  add_card++;
				  W25Q_Write_Nbytes((add_card-1)*24 + 0x210000, (uint8_t *)&write_user, sizeof(user_info_t));
				  new_card = false;
				  sendString_uart("WDOK");
				  time_break = HAL_GetTick();
			  }
			  while (write_done)
			  {
				  // ghi các thẻ hiện tại vào bộ nhớ đệm từ Block 33
				  for (uint32_t i = 0; i<number_card; i++)
				  {
					  W25Q_FastRead_address(i*24, sizeof(user_info_t), (uint8_t *)&send_user);
					  W25Q_Write_Nbytes(i*24 + 0x210000 + ((add_card*24/(16*256))+1)*0x1000, (uint8_t *)&send_user, sizeof(user_info_t));
				  }
				  // xóa bộ nhớ ban đầu
				  for (uint8_t i=0; i<(number_card)*24/(16*256)+1; i++)
				  {
					  W25Q_Erase_Sector(i);
				  }
				  uint32_t i = 0, j = 0, k = 0;
				  user_info_t old_user, new_user;

				  while (i < number_card && j < add_card)
				  {
					  W25Q_FastRead_address(i*24 + 0x210000 + ((add_card*24/(16*256))+1)*0x1000 , sizeof(user_info_t), (uint8_t *)&old_user);
					  W25Q_FastRead_address(j*24 + 0x210000, sizeof(user_info_t), (uint8_t *)&new_user);
					  if (old_user.cardID < new_user.cardID)
					  {
						  old_user.STT = k + 1;
						  W25Q_Write_Nbytes(k*24, (uint8_t *)&old_user, sizeof(user_info_t));
						  i++; // Dịch chỉ số của mảng đã lấy
					  } else if (old_user.cardID > new_user.cardID)
					  {
						  new_user.STT = k + 1;
						  W25Q_Write_Nbytes(k*24, (uint8_t *)&new_user, sizeof(user_info_t));
						  j++; // Dịch chỉ số của mảng đã lấy
					  } else if (old_user.cardID == new_user.cardID)
					  {
						  new_user.STT = k + 1;
						  W25Q_Write_Nbytes(k*24, (uint8_t *)&new_user, sizeof(user_info_t));
						  j++; // Dịch chỉ số của mảng đã lấy
						  i++;
					  }
					  k++;
				  }
				  while (i < number_card)
				  {
					  W25Q_FastRead_address(i*24 + 0x210000 + ((add_card*24/(16*256))+1)*0x1000 , sizeof(user_info_t), (uint8_t *)&old_user);
					  old_user.STT = k + 1;
					  W25Q_Write_Nbytes(k*24, (uint8_t *)&old_user, sizeof(user_info_t));
					  i++;
					  k++;
				  }
				  while (j < add_card)
				  {
					  W25Q_FastRead_address(j*24 + 0x210000, sizeof(user_info_t), (uint8_t *)&new_user);
					  new_user.STT = k + 1;
					  W25Q_Write_Nbytes(k*24, (uint8_t *)&new_user, sizeof(user_info_t));
					  j++;
					  k++;
				  }
				  for (uint32_t i=0; i<((k*24/(16*256))+3); i++)
				  {
					  W25Q_Erase_Sector(i+16*33);
				  }
				  if (send_uart_to_PC)
				  {
					  send_uart_to_PC = false;
					  sendString_uart("WCOK");
				  } else
				  {
					  sendString("W", "COK");
				  }
				  HAL_NVIC_SystemReset();
			  }

			  if ((abs(HAL_GetTick() - time_break) > Timeout_online))
			  {
				  HAL_NVIC_SystemReset();
			  }
		  }
	  }
	  if (RST_set)
	  {
		  if (RST_timer_last > 10000)
		  {
			  ip1 = 192;
			  ip2 = 168;
			  ip3 = 0;
			  ip4 = 72;
			  ip_server1 = 192;
			  ip_server2 = 168;
			  ip_server3 = 0;
			  ip_server4 = 2;
			  port_server = 6000;
			  save_data();
		  }
		  HAL_NVIC_SystemReset();
	  }
	  if (counter_reset == 720)
	  {
		  HAL_NVIC_SystemReset();
	  }
	  HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 60;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 1874;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_STT_Pin|LED_STT_ETH_Pin|LED_BP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|S_OUT2_Pin|S_OUT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_W25_GPIO_Port, CS_W25_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_STT_Pin LED_STT_ETH_Pin LED_BP_Pin */
  GPIO_InitStruct.Pin = LED_STT_Pin|LED_STT_ETH_Pin|LED_BP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BYPASS_Pin */
  GPIO_InitStruct.Pin = BYPASS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BYPASS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 S_OUT2_Pin S_OUT1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|S_OUT2_Pin|S_OUT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_W25_Pin */
  GPIO_InitStruct.Pin = CS_W25_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CS_W25_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DE_Pin */
  GPIO_InitStruct.Pin = DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RST_Pin */
  GPIO_InitStruct.Pin = RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RST_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendData_eth (char *CMD, uint32_t data)
{
	uint8_t cmd[6];
	cmd[0] = CMD[0];
	cmd[1] = data>>24&0xFF;
	cmd[2] = data>>16&0xFF;
	cmd[3] = data>>8&0xFF;
	cmd[4] = data&0xFF;
	send(SOCK_NUM,(uint8_t *) cmd, 5);
//	sendto(0, cmd, 5, server_ip,port_server);// send UDP
}
void send_u8_eth (char *CMD, uint8_t data)
{
//	char buf[10] = {0};
	uint8_t cmd[2];
	cmd[0] = CMD[0];
	cmd[1] = data;
	send(SOCK_NUM,(uint8_t *) cmd, 2);
//	sendto(0, cmd, 5, server_ip,port_server);// send UDP
}
void sendString (char *CMD, char *data)
{
//	char buf[10] = {0};
	char cmd[5];
	strcpy(cmd,CMD);
	strcat(cmd, data);
//	cmd[1] = (data&0xFF000000)>>24;
//	cmd[2] = (data&0x00FF0000)>>16;
//	cmd[3] = (data&0x0000FF00)>>8;
//	cmd[4] = (data&0x000000FF)>>0;
	send(SOCK_NUM,(uint8_t *) cmd, strlen(cmd));
//	sendto(0, cmd, 5, server_ip,port_server);// send UDP
}
void sendData_eth_info (char *CMD, uint8_t *data)
{
	char cx[27];
//	strcpy(cx,CMD);
//	strcat(cx, data);
	cx[0] = CMD[0];
	for (uint8_t i=0; i<26; i++)
	{
		cx[i+1] = data[i];
	}
	send(SOCK_NUM,(uint8_t *) cx, 27);
}
void sendData_eth_CardID (char *CMD, user_info_t user)
{
	uint8_t cx[14];
//	strcpy(cx,CMD);
//	strcat(cx, data);
	cx[0] = CMD[0];
//	cx[1] = user.STT>>24|0xFF;
//	cx[2] = user.STT>>16|0xFF;
//	cx[3] = user.STT>>8|0xFF;
//	cx[4] = user.STT>>0|0xFF;
	cx[1] = 2;
	cx[2] = user.cardID>>24&0xFF;
	cx[3] = user.cardID>>16&0xFF;
	cx[4] = user.cardID>>8&0xFF;
	cx[5] = user.cardID>>0&0xFF;
	cx[6] = user.permis[0];
	cx[7] = user.permis[1];
	cx[8] = user.permis[2];
	cx[9] = user.permis[3];
	cx[10] = user.permis[4];
	cx[11] = user.permis[5];
	cx[12] = user.permis[6];
	cx[13] = user.permis[7];
	send(SOCK_NUM,(uint8_t *) cx, 14);
}
void sendData_uart (char *CMD, uint8_t data)
{
	char cmd[2];
	cmd[0] = CMD[0];
	cmd[1] = data;
//	strcpy(cmd,CMD);
//	strcat(cmd,data);
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1,(uint8_t *) cmd, 2, 500);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
}
void sendString_uart (char *CMD)
{
	char cmd[2];
	cmd[0] = CMD[0];
	cmd[1] =  CMD[1];
	cmd[2] =  CMD[2];
	cmd[3] =  CMD[3];
//	strcpy(cmd,CMD);
//	strcat(cmd,data);
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1,(uint8_t *) cmd, 4, 500);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
}
void sendString_info_uart (uint8_t *CMD)
{
	uint8_t cmd[27];
	cmd[0] = 0x52;
	for (uint8_t i=0; i<26; i++)
	{
		cmd[i+1] =  CMD[i];
	}
//	strcpy(cmd,CMD);
//	strcat(cmd,data);
	// Pull DE high to enable TX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, cmd, 27, 1000);
	// Pull RE Low to enable RX operation
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
}
void save_data()
{
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&EraseInit, &SectorError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC00, ip1);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC04, ip2);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC08, ip3);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC0C, ip4);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC10, port_client);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC14, ip_server1);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC18, ip_server2);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC1C, ip_server3);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC20, ip_server4);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC24, port_server);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC28, time_delay);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC2C, elevator_mode);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC30, lock_default_1);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x0800FC34, lock_default_2);
	HAL_FLASH_Lock();
}

user_info_t binary_search(uint32_t Number_card, uint32_t code)
{
	uint32_t low = 1, high = Number_card;
	user_info_t user_0={0}, user_compare;

    while (low <= high) {
    	uint32_t mid = (low + high) / 2;
		W25Q_FastRead((mid-1)/16, ((mid-1)%16)*sizeof(user_info_t), sizeof(user_info_t), (uint8_t *)&user_compare);
        if (user_compare.cardID == code) {
            return user_compare;
        } else if (user_compare.cardID < code) {
            low = mid + 1;
        } else {
            high = mid - 1;
        }
    }
    return user_0;
}
void new_card_update(uint8_t *data, bool *input, uint32_t *input_timer)
{
	for (uint8_t i=0; i<8; i++)
	{
		for (uint8_t j=0; j<8; j++)
		{
			if((data[i]>>j)&0x01)//2 + i
			{
				input[i*8+j] = true;
				input_timer[i*8+j] = HAL_GetTick();
			}
		}
	}
}
void shift_left_1bit( uint8_t input[8], uint8_t output[8])
{
    uint8_t carry = 0;  // Khởi tạo carry ban đầu
    for (int i = 0; i < 8; i++)
    {
        // Lấy bit MSB của byte hiện tại trước khi dịch
        uint8_t new_carry = (input[i] >> 7) & 0x01;
        // Dịch trái 1 bit và kết hợp với carry từ byte trước
        output[i] = (input[i] << 1) | carry;
        // Cập nhật carry cho byte tiếp theo
        carry = new_carry;
    }
}
void calculate_data_can(bool *input, uint32_t *input_timer, uint8_t *data_can, uint8_t *lock_df, uint8_t delay_time, uint8_t *counter_start)
{
	uint8_t data[8] = {0};
	if (*counter_start < delay_time)
	{
		for (uint8_t i=0; i<8; i++)
		{
			for (uint8_t j=0; j<8; j++)
			{
				if (input_timer[i*8+j] == 0)
				{
					if ((lock_df[i]>>j)&0x01)
					{
						input[i*8+j] = true;
					} else
					{
						input[i*8+j] = false;
					}
				} else
				{
					*counter_start = delay_time;
					break;
				}
			}
			if (*counter_start == delay_time) break;
		}
	}
	if (*counter_start >= delay_time)
	{
		for (uint8_t i=0; i<8; i++)
		{
			for (uint8_t j=0; j<8; j++)
			{
				if (((lock_df[i]>>j)&0x01) && (abs(HAL_GetTick() - input_timer[i*8+j]) > delay_time*1000))
				{
					input[i*8+j] = true;
				} else
				{
					input[i*8+j] = false;
				}
			}
		}
	}
	for (uint8_t i=0; i<8; i++)
	{
		for (uint8_t j=0; j<8; j++)
		{
			data[i] |= input[i*8+j]<<j;
		}
	}
	shift_left_1bit(data, data_can);
}
uint8_t reconect_eth(uint8_t sn)
{
//	bool linkport = false;
	uint8_t Status_SN;
	Status_SN = getSn_SR(sn);
	if (wizphy_getphylink() == PHY_LINK_OFF)
	{
		HAL_GPIO_WritePin(LED_STT_ETH_GPIO_Port, LED_STT_ETH_Pin, GPIO_PIN_RESET);
//		close(sn);
		disconnect(sn);
	}
	if (Status_SN == SOCK_CLOSE_WAIT)
	{
		HAL_GPIO_WritePin(LED_STT_ETH_GPIO_Port, LED_STT_ETH_Pin, GPIO_PIN_RESET);
//		close(sn);
		disconnect(sn);
	}
	if (wizphy_getphylink() == PHY_LINK_ON && Status_SN == SOCK_CLOSED)
	{

		HAL_GPIO_WritePin(LED_STT_ETH_GPIO_Port, LED_STT_ETH_Pin, GPIO_PIN_RESET);
		socket(sn, Sn_MR_TCP, port_client, SF_TCP_NODELAY);
		connect(sn, server_ip, port_server);
		counter_reset++;
	}
	Status_SN = getSn_SR(sn);
	if (Status_SN == SOCK_ESTABLISHED)
	{
		HAL_GPIO_WritePin(LED_STT_ETH_GPIO_Port, LED_STT_ETH_Pin, GPIO_PIN_SET);
	}
	return Status_SN;
}

void Set_speed_can(uint8_t speed)
{
	if (speed == 0)// 100kps
	{
		hcan.Init.Prescaler = 30;
		hcan.Init.TimeSeg1 = CAN_BS1_8TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
	} else if (speed == 1)// 50kps
	{
		hcan.Init.Prescaler = 40;
		hcan.Init.TimeSeg1 = CAN_BS1_12TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_5TQ;
	} else if (speed == 2)// 25kps
	{
		hcan.Init.Prescaler = 60;
		hcan.Init.TimeSeg1 = CAN_BS1_16TQ;
		hcan.Init.TimeSeg2 = CAN_BS2_7TQ;
	}
	hcan.Init.AutoRetransmission = ENABLE;
	hcan.Init.SyncJumpWidth = CAN_SJW_3TQ;
	HAL_CAN_Init(&hcan);

	CAN_FilterTypeDef canfilterconfig;
	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x740<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0xFFF<<5;
	canfilterconfig.FilterMaskIdLow = 6;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 13;
	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

	HAL_CAN_Start(&hcan);
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


// Hàm kiểm tra năm nhuận
static uint8_t is_leap_year(uint16_t year)
{
    if (year % 400 == 0) {
        return 1;
    }
    if (year % 100 == 0) {
        return 0;
    }
    if (year % 4 == 0) {
        return 1;
    }
    return 0;
}

// Hàm tùy chỉnh tương tự mktime
uint32_t mktime(uint16_t year, uint8_t month, uint8_t date, uint8_t hour, uint8_t minute)
{
	uint32_t time = 0;
	uint32_t i = 0;

    // 1. Cộng dồn số giây từ các năm đã qua
    // Giả sử epoch là 1/1/2000
    for (i = EPOCH_YEAR; i < (year - EPOCH_YEAR); i++) {
        time += (365 + is_leap_year(i)) * 24 * 60;
    }

    // 2. Cộng dồn số giây từ các tháng đã qua trong năm hiện tại
    for (i = 0; i < month; i++) {
        time += days_in_month[i] * 24 * 60;
        // Cộng thêm một ngày nếu là tháng 2 của năm nhuận
        if (i == 1 && is_leap_year(year)) {
        	time += 24 * 3600;
        }
    }

    // 3. Cộng dồn số giây từ các ngày, giờ, phút và giây
    time += (uint32_t)(date - 1) * 24 * 60;
    time += (uint32_t)hour * 60;
    time += (uint32_t)minute;

    return time;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
