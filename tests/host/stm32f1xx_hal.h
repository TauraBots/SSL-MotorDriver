#pragma once
#include <stdint.h>
typedef struct { uint32_t CCER,BDTR,CNT,ARR,compare[4]; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { int unused; } ADC_HandleTypeDef;
typedef struct { int unused; } GPIO_TypeDef;
typedef struct { int unused; } I2C_HandleTypeDef;
extern TIM_TypeDef host_tim1,host_tim8;
extern uint32_t host_tick,host_adc;
#define TIM1 (&host_tim1)
#define TIM8 (&host_tim8)
#define TIM_BDTR_MOE 1U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U
#define TIM_CHANNEL_ALL 4U
#define HAL_OK 0
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->Instance->compare[c]=(v))
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_MOE_ENABLE(h) ((h)->Instance->BDTR |= TIM_BDTR_MOE)
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline uint32_t HAL_GetTick(void) { return host_tick; }
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
static inline int HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
static inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
static inline int HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*) { return HAL_OK; }
static inline int HAL_ADC_Start(ADC_HandleTypeDef*) { return HAL_OK; }
static inline int HAL_ADC_Stop(ADC_HandleTypeDef*) { return HAL_OK; }
static inline int HAL_ADC_PollForConversion(ADC_HandleTypeDef*,uint32_t) { return HAL_OK; }
static inline uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef*) { return host_adc; }
static inline void HAL_GPIO_TogglePin(GPIO_TypeDef*,uint16_t) {}
