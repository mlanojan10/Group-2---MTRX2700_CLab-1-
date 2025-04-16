#ifndef __TIMER
#define __TIMER

#include <stdint.h>
#include "stm32f303xc.h"

void enable_clocks();

void initialise_board();

void trigger_prescaler(TIM_TypeDef *TIM);

void init_timer_module(TIM_TypeDef *TIM, void (*timer_callback)());

void change_pattern();

void TIM2_IRQHandler(void);

void enable_timer2_interrupt();

uint32_t get_timer_period(TIM_TypeDef *TIM);

void set_timer_period(TIM_TypeDef *TIM, uint32_t new_period);

void reset_timer(TIM_TypeDef *TIM, uint32_t new_period);

void one_shot(TIM_TypeDef *TIM, uint32_t delay);

#endif
