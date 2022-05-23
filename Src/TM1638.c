#include "stdbool.h"
#include "TM1638.h"
#include "main.h"
#include "cmsis_os.h"

unsigned char TM1638_Buf[18], TM1638_LED_Buf[16];

void TM1638_Init()
{
	TM1638_Buf[0] = 0x8F;
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, TM1638_Buf, 1, 1000);
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
}

void TM1638_Clear()
{
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	TM1638_Buf[0] = 0x40;
	TM1638_Buf[1] = 0xC0;
	TM1638_Buf[2] = 0x00;
	TM1638_Buf[3] = 0x00;
	TM1638_Buf[4] = 0x00;
	TM1638_Buf[5] = 0x00;
	TM1638_Buf[6] = 0x00;
	TM1638_Buf[7] = 0x00;
	TM1638_Buf[8] = 0x00;
	TM1638_Buf[9] = 0x00;
	TM1638_Buf[10] = 0x00;
	TM1638_Buf[11] = 0x00;
	TM1638_Buf[12] = 0x00;
	TM1638_Buf[13] = 0x00;
	TM1638_Buf[14] = 0x00;
	TM1638_Buf[15] = 0x00;
	TM1638_Buf[16] = 0x00;
	TM1638_Buf[17] = 0x00;
	HAL_SPI_Transmit(&hspi2, TM1638_Buf, 18, 1000);
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
}

void TM1638_Read()
{
	unsigned int i;
	
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	TM1638_Buf[0] = 0x42;
	HAL_SPI_Transmit(&hspi2, TM1638_Buf, 1, 1000);
	HAL_SPI_Receive(&hspi2, TM1638_Buf, 4, 1000);
	for(i = 0; i < 50; i++)
	{
		__nop();
	}
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
}
/*
void TM1638_Read()
{
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	TM1638_Send_Buf[0] = 0x42;
	SPI1_busy = true;
	HAL_SPI_Transmit_DMA(&hspi1, TM1638_Send_Buf, 1);
	while(SPI1_busy) osThreadYield();
	SPI1_busy = true;
	HAL_SPI_Receive_DMA(&hspi1, TM1638_Rec_Buf, 4);
	while(SPI1_busy) osThreadYield();
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	TM1638_Send_Buf[0] = 0x40;
	TM1638_Send_Buf[1] = 0xC0;
	TM1638_Send_Buf[2] = TM1638_LED_Buf[0];
	TM1638_Send_Buf[3] = TM1638_LED_Buf[1];
	TM1638_Send_Buf[4] = TM1638_LED_Buf[2];
	TM1638_Send_Buf[5] = TM1638_LED_Buf[3];
	TM1638_Send_Buf[6] = TM1638_LED_Buf[4];
	TM1638_Send_Buf[7] = TM1638_LED_Buf[5];
	TM1638_Send_Buf[8] = TM1638_LED_Buf[6];
	TM1638_Send_Buf[9] = TM1638_LED_Buf[7];
	TM1638_Send_Buf[10] = TM1638_LED_Buf[8];
	TM1638_Send_Buf[11] = TM1638_LED_Buf[9];
	TM1638_Send_Buf[12] = TM1638_LED_Buf[10];
	TM1638_Send_Buf[13] = TM1638_LED_Buf[11];
	TM1638_Send_Buf[14] = TM1638_LED_Buf[12];
	TM1638_Send_Buf[15] = TM1638_LED_Buf[13];
	TM1638_Send_Buf[16] = TM1638_LED_Buf[14];
	TM1638_Send_Buf[17] = TM1638_LED_Buf[15];
	SPI1_busy = true;
	HAL_SPI_Transmit_DMA(&hspi1, TM1638_Send_Buf, 18);
	while(SPI1_busy) osThreadYield();
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
}

void TM1638_Send()
{
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_RESET);
	TM1638_Send_Buf[0] = 0x40;
	TM1638_Send_Buf[1] = 0xC0;
	TM1638_Send_Buf[2] = 0xFF;
	TM1638_Send_Buf[3] = 0xFF;
	TM1638_Send_Buf[4] = 0xFF;
	TM1638_Send_Buf[5] = 0xFF;
	TM1638_Send_Buf[6] = 0xFF;
	TM1638_Send_Buf[7] = 0xFF;
	TM1638_Send_Buf[8] = 0xFF;
	TM1638_Send_Buf[9] = 0xFF;
	TM1638_Send_Buf[10] = 0xFF;
	SPI1_busy = true;
	HAL_SPI_Transmit_DMA(&hspi1, TM1638_Send_Buf, 10);
	while(SPI1_busy) osThreadYield();
	HAL_GPIO_WritePin(TM_STB_GPIO_Port, TM_STB_Pin, GPIO_PIN_SET);
}
*/
