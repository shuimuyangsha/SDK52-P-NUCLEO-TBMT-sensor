
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"

#include "r3_1_f30x_pwm_curr_fdbk.h"
 
 

  #define MAX_TWAIT 0                 /* Dummy value for single drive */
  #define FREQ_RATIO 1                /* Dummy value for single drive */
  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Motor 1 - three shunt 1 ADC (STM32F302x8)
  */
const R3_1_F30XParams_t R3_1_F30XParamsM1 =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .ADCx = ADC1,     
  .bIaChannel = MC_ADC_CHANNEL_1,                 
  .bIbChannel = MC_ADC_CHANNEL_7,                 
  .bIcChannel = MC_ADC_CHANNEL_6,                 
                                        
/* PWM generation parameters --------------------------------------------------*/
  .bRepetitionCounter = REP_COUNTER,                       
  .hTafter            = TW_AFTER,                          
  .hTbefore           = TW_BEFORE_R3_1,                    
  .TIMx               = PWM_TIM1,               
                                     
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs = (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING, 
  .pwm_en_u_port = M1_PWM_EN_U_GPIO_Port,                                 
  .pwm_en_u_pin  = M1_PWM_EN_U_Pin,                    
  .pwm_en_v_port = M1_PWM_EN_V_GPIO_Port,                   
  .pwm_en_v_pin  = M1_PWM_EN_V_Pin,                    
  .pwm_en_w_port = M1_PWM_EN_W_GPIO_Port,                   
  .pwm_en_w_pin  = M1_PWM_EN_W_Pin,   

/* Emergency input (BKIN2) signal initialization -----------------------------*/
  .bBKIN2Mode = EXT_MODE,                        
                                     
/* Internal COMP settings ----------------------------------------------------*/
  .wCompOCPASelection     = MC_NULL,
  .bCompOCPAInvInput_MODE = MC_NULL,                          
  .wCompOCPBSelection     = MC_NULL,      
  .bCompOCPBInvInput_MODE = MC_NULL,              
  .wCompOCPCSelection     = MC_NULL,        
  .bCompOCPCInvInput_MODE = MC_NULL,                                         

  .wCompOVPSelection      = MC_NULL,       
  .bCompOVPInvInput_MODE  = MC_NULL,
                                      
/* DAC settings --------------------------------------------------------------*/
  .hDAC_OCP_Threshold = 23830,                            
  .hDAC_OVP_Threshold =  23830,                            
                                     
/* Regular conversion --------------------------------------------------------*/
  .regconvADCx =  ADC1                         
                                     
};
   

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
