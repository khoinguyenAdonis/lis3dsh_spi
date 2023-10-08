/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_6|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV16;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/* USER CODE BEGIN 1 */
#define WHO_AM_I           (0x0F)
#define SPI_CS_LOW()      do{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, 0);} while(0) ;
#define SPI_CS_HIGH()     do{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, 1);} while(0) ;
#define DEFAUL             (0x3B)
void SPI_Init()
{
    LL_SPI_Enable(SPI1);
    HAL_Delay(100);
}
uint8_t Transmit_1byte( uint8_t int_reg)
{
  while (!LL_SPI_IsActiveFlag_TXE(SPI1)){};
  LL_SPI_TransmitData8(SPI1,int_reg);
  // read request
  while (!LL_SPI_IsActiveFlag_RXNE(SPI1)){};
  return LL_SPI_ReceiveData8(SPI1);
}

uint8_t  spi_read_addr(uint8_t reg_adr)
{
  uint8_t uid = 0;
  uint8_t command= 0x80 | reg_adr  ; 
  // ctr cs 
  SPI_CS_LOW();
  // write command
  Transmit_1byte(command);
  // get address sensor
  uid =  Transmit_1byte(0x00);
  // disable ctr cs
  SPI_CS_HIGH();
  return uid;
}
void Init_Sensor()
{
  uint8_t command= 0x27; // 100111 = 27(hex) ctrl_reg1 dr = 0 , pd = 1, fs = 0, stp/stm = 0, z = 1,x = 1, y=1 
  // ctr cs 
  SPI_CS_LOW();
  // write command
  Transmit_1byte(command);
  SPI_CS_HIGH();
}
uint8_t DataBuff[6] = {0};

void Sensor_read_data(uint8_t* user_buffer, uint32_t len)
{
  uint8_t command= 0xC0 | 0x29  ; // multiple byte read = 1 Ms = 1 bit(2-7) = 0 -> 11000000 = 0XC0 ; address first 0x29  
  // ctr cs 
  SPI_CS_LOW();
  // write command
  Transmit_1byte(command);
  // 
  for(uint32_t i =0; i< len ; i++)
  {
    *user_buffer++ = Transmit_1byte(0x00);
  }
  // disable ctr cs
  SPI_CS_HIGH();
}

static uint8_t id = 0 ;
void uid_test(void)
{
  SPI_Init();
  id = spi_read_addr(WHO_AM_I);
  if (id != DEFAUL){return;}
  Init_Sensor();
  HAL_Delay(100);
  while(1)
  {
    Sensor_read_data(DataBuff[0],6);
  }
}
/* USER CODE END 1 */
