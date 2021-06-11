/*
 * systick.c
 *
 *  Created on: 4 июн. 2021 г.
 *      Author: sa100
 */

#include "systick.h"
#include "cmsis_device.h"

/* --- private defines ---------------------------------------------------- */
#define FREQUENCY_HZ 		1000UL


/* --- private variables -------------------------------------------------- */
static volatile uint32_t uwTick;
static volatile uint32_t uptime;


void
SysTick_Init()
{
  // Use SysTick as reference for the delay loops.
  SysTick_Config(SystemCoreClock / FREQUENCY_HZ);
}

uint32_t
get_tick()
{
  return uwTick;
}


uint32_t
get_uptime()
{
  return uptime;
}

__attribute__((weak))
void on_tick_callback()
{
}

__attribute__((weak))
void every_second_callback()
{
}

void
SysTick_Handler()
{
  uwTick++;
  on_tick_callback();

  if ((uwTick % FREQUENCY_HZ) == 0)
  {
    uptime++;
    every_second_callback();
  }
}
