/*
 * main.c
 */

#include <stdio.h>

#include "main.h"
#include "usb_lib.h"

/* --- private defines ---------------------------------------------------- */

/* --- private variables -------------------------------------------------- */

/* string buffer */
char str[128];


/**
 *
 */
void main()
{
  SysTick_Init();
  USB_Init();

  for(;;)
  {
  }
}

/**
 *
 */
void every_second_callback()
{
}

/**
 *
 */
void on_tick_callback()
{
}

/**
 *
 */
void USB_HardwareInit()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_StructInit(&GPIO_InitStructure);

  /* use PLL clock */
  RCC->CFGR &= ~(1 << 15); // CLK48SEL
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_Div1);

  /* Enable USB Clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  /* GPIO */
  RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // Analog input
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* NVIC */
  NVIC_InitStructure.NVIC_IRQChannel = USB_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_Init(&NVIC_InitStructure);
}

/**
 *
 */
void VCOM_onDataReceivedFromHost(uint8_t *data, size_t size)
{
  if (data[size - 1] == '\n') {
    data[size - 1] = '~';
  }

  snprintf(str, sizeof(str), "received: '%s' %d bytes\n", data, size);
  VCOM_sendDataToHost((uint8_t*)str, strlen(str));
}

