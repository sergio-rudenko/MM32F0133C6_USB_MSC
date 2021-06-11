/*
 * SysTickTimer.h
 *
 *  Created on: 4 июн. 2021 г.
 *      Author: sa100
 */

#ifndef SYSTICKTIMER_H_
#define SYSTICKTIMER_H_

#include <stdint.h>

/* ---[ function prototypes ]---------------------------------------------- */
#ifdef __cplusplus
extern "C" {
#endif

  void 		SysTick_Init();

  uint32_t 	get_tick();
  uint32_t 	get_uptime();

  void 		on_tick_callback();
  void		every_second_callback();

#ifdef __cplusplus
}
#endif


#endif /* SYSTICKTIMER_H_ */
