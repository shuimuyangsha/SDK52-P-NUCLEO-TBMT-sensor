/**
  ******************************************************************************
  * @file    parameters_conversion_f30x.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F3 Family.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F30X_H
#define __PARAMETERS_CONVERSION_F30X_H

#include "pmsm_motor_parameters.h"
#include "drive_parameters.h"
#include "power_stage_parameters.h"
#include "mc_math.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      72000000uL
#define TIM_CLOCK_DIVIDER  1 
#define ADV_TIM_CLK_MHz  72
#define ADC_CLK_MHz     72
#define HALL_TIM_CLK    72000000uL

/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  

/*************************  IRQ Handler Mapping  *********************/
#define TIMx_UP_M1_IRQHandler TIM1_UP_TIM16_IRQHandler

#define TIMx_BRK_M1_IRQHandler TIM1_BRK_TIM15_IRQHandler

#define TRIG_CONV_LATENCY_NS	(7000uL/(2*ADC_CLK_MHz))
#define SAMPLING_TIME_NS (((19 * 1000uL)+500)/ADC_CLK_MHz)
#define TW_BEFORE (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS + TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+1u)

#define ADC_CONV_NB_CK 13u
#define TW_BEFORE_R3_1 (((uint16_t)(((((uint16_t)(SAMPLING_TIME_NS * 2 + TRIG_CONV_LATENCY_NS)))*ADV_TIM_CLK_MHz)/1000ul))+ 1u + ADC_CONV_NB_CK)

#define M1_VBUS_SW_FILTER_BW_FACTOR      6u

#define M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_61CYCLES_5

#define M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_61CYCLES_5

#define MCU_SUPPLY_VOLTAGE  3.30
#define DAC_VMAX ((float)(MCU_SUPPLY_VOLTAGE))
#define DAC_MAX_DIGIT 65536.0f

#define OCP_OFFSET (((float)(OCP_V)) * (OCP_R1/(OCP_R1 + OCP_R2)))
#define OCP_TH_V (OCP_OFFSET + ((float)(RSHUNT)*OCP_THRESHOLD)*(OCP_R2/(OCP_R1+OCP_R2)))
#define OCP_DAC_VREF (uint16_t)((OCP_TH_V * DAC_MAX_DIGIT)/DAC_VMAX)
#define OVP_DAC_VREF (uint16_t)(((float)(OV_VOLTAGE_THRESHOLD_V)*(float)(VBUS_PARTITIONING_FACTOR)*DAC_MAX_DIGIT)/DAC_VMAX)

#define OPAMP1_InvertingInput_PC5         LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP1_InvertingInput_PA3         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP1_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP1_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP2_InvertingInput_PC5         LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP2_InvertingInput_PA5         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP2_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP2_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP3_InvertingInput_PB10        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP3_InvertingInput_PB2         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP3_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP3_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER
#define OPAMP4_InvertingInput_PB10        LL_OPAMP_INPUT_INVERT_IO0
#define OPAMP4_InvertingInput_PD8         LL_OPAMP_INPUT_INVERT_IO1 
#define OPAMP4_InvertingInput_PGA         LL_OPAMP_INPUT_INVERT_CONNECT_NO
#define OPAMP4_InvertingInput_FOLLOWER    LL_OPAMP_MODE_FOLLOWER

#define OPAMP1_NonInvertingInput_PA7      LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP1_NonInvertingInput_PA5      LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP1_NonInvertingInput_PA3      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP1_NonInvertingInput_PA1      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP2_NonInvertingInput_PD14     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP2_NonInvertingInput_PB14     LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP2_NonInvertingInput_PB0      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP2_NonInvertingInput_PA7      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP3_NonInvertingInput_PB13     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP3_NonInvertingInput_PA5      LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP3_NonInvertingInput_PA1      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP3_NonInvertingInput_PB0      LL_OPAMP_INPUT_NONINVERT_IO0
#define OPAMP4_NonInvertingInput_PD11     LL_OPAMP_INPUT_NONINVERT_IO1
#define OPAMP4_NonInvertingInput_PB11     LL_OPAMP_INPUT_NONINVERT_IO3
#define OPAMP4_NonInvertingInput_PA4      LL_OPAMP_INPUT_NONINVERT_IO2
#define OPAMP4_NonInvertingInput_PB13     LL_OPAMP_INPUT_NONINVERT_IO0

#define OPAMP1_PGAConnect_PC5             OPAMP_CSR_PGGAIN_3
#define OPAMP1_PGAConnect_PA3             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP2_PGAConnect_PC5             OPAMP_CSR_PGGAIN_3
#define OPAMP2_PGAConnect_PA5             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP3_PGAConnect_PB10            OPAMP_CSR_PGGAIN_3
#define OPAMP3_PGAConnect_PB2             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)
#define OPAMP4_PGAConnect_PB10            OPAMP_CSR_PGGAIN_3
#define OPAMP4_PGAConnect_PD8             (OPAMP_CSR_PGGAIN_3|OPAMP_CSR_PGGAIN_2)

#define COMP1_InvertingInput_PA0          LL_COMP_INPUT_MINUS_IO1
#define COMP2_InvertingInput_PA2          LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PD15         LL_COMP_INPUT_MINUS_IO1
#define COMP3_InvertingInput_PB12         LL_COMP_INPUT_MINUS_IO2
#define COMP4_InvertingInput_PE8          LL_COMP_INPUT_MINUS_IO1
#define COMP4_InvertingInput_PB2          LL_COMP_INPUT_MINUS_IO2
#define COMP5_InvertingInput_PD13         LL_COMP_INPUT_MINUS_IO1
#define COMP5_InvertingInput_PB10         LL_COMP_INPUT_MINUS_IO2
#define COMP6_InvertingInput_PD10         LL_COMP_INPUT_MINUS_IO1
#define COMP6_InvertingInput_PB15         LL_COMP_INPUT_MINUS_IO2
#define COMP7_InvertingInput_PC0          LL_COMP_INPUT_MINUS_IO1

#define COMPX_InvertingInput_DAC1        LL_COMP_INPUT_MINUS_DAC1_CH1
#define COMPX_InvertingInput_DAC2        LL_COMP_INPUT_MINUS_DAC1_CH2
#define COMPX_InvertingInput_VREF        LL_COMP_INPUT_MINUS_VREFINT
#define COMPX_InvertingInput_VREF_1_4    LL_COMP_INPUT_MINUS_1_4VREFINT
#define COMPX_InvertingInput_VREF_1_2    LL_COMP_INPUT_MINUS_1_2VREFINT
#define COMPX_InvertingInput_VREF_3_4    LL_COMP_INPUT_MINUS_3_4VREFINT

#define COMP1_NonInvertingInput_PA1    LL_COMP_INPUT_PLUS_IO1
#define COMP2_NonInvertingInput_PA3    LL_COMP_INPUT_PLUS_IO2
#define COMP2_NonInvertingInput_PA7    LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PB14   LL_COMP_INPUT_PLUS_IO1
#define COMP3_NonInvertingInput_PD14   LL_COMP_INPUT_PLUS_IO2
#define COMP4_NonInvertingInput_PB0    LL_COMP_INPUT_PLUS_IO1
#define COMP4_NonInvertingInput_PE7    LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PB13   LL_COMP_INPUT_PLUS_IO2
#define COMP5_NonInvertingInput_PD12   LL_COMP_INPUT_PLUS_IO1
#define COMP6_NonInvertingInput_PB11   LL_COMP_INPUT_PLUS_IO2
#define COMP6_NonInvertingInput_PD11   LL_COMP_INPUT_PLUS_IO1
#define COMP7_NonInvertingInput_PC1    LL_COMP_INPUT_PLUS_IO2
#define COMP7_NonInvertingInput_PA0    LL_COMP_INPUT_PLUS_IO1

#endif /*__PARAMETERS_CONVERSION_F30X_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
