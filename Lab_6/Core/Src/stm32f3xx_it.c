/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdlib.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef unsigned char byte;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int brightness = 0, brightness_count = 0, brightness_sum = 0, brightness_percentage = 0;
int temperature = 0, temperature_last = 0, temperature_count = 0, temperature_sum = 0;
int buzzer_state = 0, buzzer_counter = 0;
int brightness_state = 0, brightness_state_count = 0;
int pir_state = 0, pir_counter = 0;
int usart_state = 0, usart_count = 0, usart_show = 0;

extern byte blank[];
extern byte lamp[];
extern byte left[];
extern byte middle[];
extern byte right[];
extern byte full[];
extern byte thermometer[];
extern byte degree[];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern TIM_HandleTypeDef *pwm_timer;
extern uint32_t pwm_channel;

void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) {
    if (pwm_freq == 0 || pwm_freq > 20000) {
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, 0);
    } else {
        const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
        // const uint16_t prescaler = 1;
        const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
        const uint32_t timer_clock = internal_clock_freq / prescaler;
        const uint32_t period_cycles = timer_clock / pwm_freq;
        const uint32_t pulse_width = volume * period_cycles / 1000 / 2;

        pwm_timer->Instance->PSC = prescaler - 1;
        pwm_timer->Instance->ARR = period_cycles - 1;
        pwm_timer->Instance->EGR = TIM_EGR_UG;
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
    }
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */
extern RTC_TimeTypeDef now_time;
extern RTC_DateTypeDef now_date;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    while (1) {
    }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F3xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f3xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 interrupts.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */

  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
    int val = HAL_ADC_GetValue(&hadc1);

    if (brightness_count < 5) {
        brightness_sum += val;
        brightness_count++;
    } else if (brightness_count == 5) {
        brightness = ((brightness_sum / 5) * 100 / 1024);

        brightness_count = 0;
        brightness_sum = 0;

        if (pir_state == 0) {
            createChar(0, lamp);
            createChar(1, left);
            createChar(2, middle);
            createChar(3, right);
            createChar(4, full);
            createChar(5, blank);

            setCursor(1, 0);
            write(0);
            setCursor(13, 0);
            write(5);
            setCursor(14, 0);
            write(5);

            int cursor = 2;

            if (brightness < 10) {
                setCursor(cursor, 0);
                cursor++;
                write(1);

                for (int i = 0; i < 8 - (brightness / 10); i++) {
                    setCursor(cursor, 0);
                    cursor++;
                    write(2);
                }

                setCursor(cursor, 0);
                write(3);

                cursor = 15;
            } else if (brightness < 100) {
                for (int i = 0; i < brightness / 10; i++) {
                    setCursor(cursor, 0);
                    cursor++;
                    write(4);
                }

                for (int i = 0; i <= 8 - (brightness / 10); i++) {
                    setCursor(cursor, 0);
                    cursor++;
                    write(2);
                }

                setCursor(cursor, 0);
                write(3);

                cursor = 14;
            } else {
                for (int i = 0; i < 10; i++) {
                    setCursor(cursor, 0);
                    cursor++;
                    write(4);
                }

                cursor = 13;
            }

            setCursor(cursor, 0);
            char brightness_str[5];
            sprintf(brightness_str, "%d%%", brightness);
            print(brightness_str);
        }
    }
  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_Start_IT(&hadc4);

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) && pir_state == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        pir_state = 1;
        usart_state = 1;

        begin(20, 4);

        setCursor(9, 0);
        print("!!!");

        setCursor(3, 2);
        HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &now_date, RTC_FORMAT_BIN);

        char time_date_str[100];
        sprintf(time_date_str, "%d:%d:%d %d|%d|%d", now_time.Hours, now_time.Minutes, now_time.Seconds, now_date.Year,
                now_date.Month, now_date.Date);
        print(time_date_str);
    }

    if (pir_state == 1 && pir_counter % 100 == 0) {
        setCursor(7, 2);
        HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN);

        char second_str[100];
        sprintf(second_str, "%d", now_time.Seconds);
        print(second_str);
    }

    if (pir_state == 1) {
        pir_counter++;
    }

    if (pir_counter == 300) {
        begin(20, 4);
        pir_state = 0;
        pir_counter = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
    }


    if (abs(temperature_last - temperature) >= 1 && buzzer_state == 0) {
        buzzer_state = 1;
        usart_state = 1;
        PWM_Change_Tone(500, 100);
    }

    if (buzzer_state == 1) {
        buzzer_counter++;
    }

    if (buzzer_counter == 10) {
        PWM_Change_Tone(500, 0);
        buzzer_state = 0;
        buzzer_counter = 0;
    }


    if (brightness < 20 || brightness > 80) {
        usart_state = 1;
        brightness_state = 1;
    }


    if (usart_state == 1) {
        HAL_RTC_GetTime(&hrtc, &now_time, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &now_date, RTC_FORMAT_BIN);

        char time_date_str[100];
        int message_length = 0;

        if (pir_state == 1) {
            message_length = sprintf(time_date_str, "%d:%d:%d %d|%d|%d : Move\n", now_time.Hours, now_time.Minutes,
                                     now_time.Seconds, now_date.Year, now_date.Month, now_date.Date);
        }

        HAL_UART_Transmit(&huart2, time_date_str, message_length, 1000);

        if (buzzer_state == 1) {
            message_length = sprintf(time_date_str, "%d:%d:%d %d|%d|%d : Temperature\n", now_time.Hours,
                                     now_time.Minutes, now_time.Seconds, now_date.Year, now_date.Month, now_date.Date);
        }

        HAL_UART_Transmit(&huart2, time_date_str, message_length, 1000);

        if (brightness_state == 1) {
            message_length = sprintf(time_date_str, "%d:%d:%d %d|%d|%d : Brightness\n", now_time.Hours,
                                     now_time.Minutes, now_time.Seconds, now_date.Year, now_date.Month, now_date.Date);
            brightness_state = 0;
        }

        HAL_UART_Transmit(&huart2, time_date_str, message_length, 1000);
        usart_state = 0;
        usart_show = 0;
    }
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles ADC4 interrupt.
  */
void ADC4_IRQHandler(void)
{
  /* USER CODE BEGIN ADC4_IRQn 0 */

  /* USER CODE END ADC4_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc4);
  /* USER CODE BEGIN ADC4_IRQn 1 */
    int val = HAL_ADC_GetValue(&hadc4);

    if (temperature_count < 5) {
        temperature_sum += val;
        temperature_count++;
    } else if (temperature_count == 5) {
        temperature_last = temperature;
        temperature = (((int) temperature_sum / 5) * 330 / 4095);
        temperature_count = 0;
        temperature_sum = 0;

        if (pir_state == 0) {
            createChar(6, thermometer);
            createChar(7, degree);

            setCursor(1, 2);
            write(6);

            setCursor(3, 2);
            char temperature_str[5];
            sprintf(temperature_str, "%d", temperature);
            print(temperature_str);

            setCursor(5, 2);
            write(7);

            setCursor(6, 2);
            print("C");

            setCursor(12, 2);

            if (temperature < 10) {
                print("!Cold!");
            } else if (temperature < 25) {
                print("!Mild!");
            } else {
                print("!Warm!");
            }
        }
    }
  /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
