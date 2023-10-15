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
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int seven_segment_state = 0, buzzer_state = 0, buzzer_count = 0;
int volume = 0, vol_uart_count = 0;
float brightness = 0;
int brightness_count = 0, brightness_sum = 0;
float temperature = 0, temperature_last = 0;
int temperature_count = 0, temperature_sum = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void show_number_on_seven_segment(int);

void digits_off();

void digits_on();

void ic_input_value(int);

int BCD_Convertor(int number);
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
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim1;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void) {
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
void HardFault_Handler(void) {
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void) {
    /* USER CODE BEGIN MemoryManagement_IRQn 0 */

    /* USER CODE END MemoryManagement_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
        /* USER CODE END W1_MemoryManagement_IRQn 0 */
    }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void) {
    /* USER CODE BEGIN BusFault_IRQn 0 */

    /* USER CODE END BusFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_BusFault_IRQn 0 */
        /* USER CODE END W1_BusFault_IRQn 0 */
    }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void) {
    /* USER CODE BEGIN UsageFault_IRQn 0 */

    /* USER CODE END UsageFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
        /* USER CODE END W1_UsageFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void) {
    /* USER CODE BEGIN SVCall_IRQn 0 */

    /* USER CODE END SVCall_IRQn 0 */
    /* USER CODE BEGIN SVCall_IRQn 1 */

    /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void) {
    /* USER CODE BEGIN DebugMonitor_IRQn 0 */

    /* USER CODE END DebugMonitor_IRQn 0 */
    /* USER CODE BEGIN DebugMonitor_IRQn 1 */

    /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void) {
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void) {
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
void ADC1_2_IRQHandler(void) {
    /* USER CODE BEGIN ADC1_2_IRQn 0 */

    /* USER CODE END ADC1_2_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc1);
    /* USER CODE BEGIN ADC1_2_IRQn 1 */
    int val = HAL_ADC_GetValue(&hadc1);

    if (brightness_count < 5) {
        brightness_sum += val;
        brightness_count++;
    } else if (brightness_count == 5) {
        brightness = (((float) brightness_sum / 5) * 100 / 4000);
        brightness_count = 0;
        brightness_sum = 0;
    }
    /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void) {
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_Start_IT(&hadc3);
    HAL_ADC_Start_IT(&hadc4);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (int) brightness);
    /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void) {
    /* USER CODE BEGIN TIM4_IRQn 0 */

    /* USER CODE END TIM4_IRQn 0 */
    HAL_TIM_IRQHandler(&htim4);
    /* USER CODE BEGIN TIM4_IRQn 1 */
    digits_off();

    show_number_on_seven_segment(seven_segment_state);
    seven_segment_state++;
    seven_segment_state %= 4;

    /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void) {
    /* USER CODE BEGIN USART2_IRQn 0 */

    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */

    /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void) {
    /* USER CODE BEGIN ADC3_IRQn 0 */

    /* USER CODE END ADC3_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc3);
    /* USER CODE BEGIN ADC3_IRQn 1 */
    int val = HAL_ADC_GetValue(&hadc3);

    float temp = ((float) val * 1500 / 4095) + 500;
    volume = temp;
    volume /= 50;
    volume *= 50;
    /* USER CODE END ADC3_IRQn 1 */
}

/**
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void) {
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

    /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */
    if (vol_uart_count % (volume / 50) == 0) {
        unsigned char data[100] = "";
        int message_length = sprintf(data, "T = %.4f - B = %.4f - V = %d\n", temperature, brightness, volume);
//        HAL_UART_Transmit(&huart2, data, message_length, 1000);

//        if (abs((int) temperature - (int) temperature_last) >= 1 && buzzer_state == 0) {
//            buzzer_state = 1;
//        }
    }
    vol_uart_count++;

//    if (buzzer_state == 3) {
//        PWM_Change_Tone(500, 0);
//        buzzer_state = 0;
//    } else if (buzzer_state == 2) {
//        buzzer_count++;
//    } else if (buzzer_state == 1) {
//        PWM_Change_Tone(500, 100);
//        buzzer_state = 2;
//    }
//
//    if (buzzer_count == 40) {
//        buzzer_state = 3;
//        buzzer_count = 0;
//    }
    /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles ADC4 interrupt.
  */
void ADC4_IRQHandler(void) {
    /* USER CODE BEGIN ADC4_IRQn 0 */

    /* USER CODE END ADC4_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc4);
    /* USER CODE BEGIN ADC4_IRQn 1 */
    int val = HAL_ADC_GetValue(&hadc4);

    unsigned char data[100] = "";
    int message_length = sprintf(data, "LM35: %d\n", val/7);
    HAL_UART_Transmit(&huart2, data, message_length, 1000);

    if (temperature_count < 5) {
        temperature_sum += val;
        temperature_count++;
    } else if (temperature_count == 5) {
        temperature_last = temperature;
        temperature = (((float) temperature_sum / 5) * 330 / 4095);
        temperature_count = 0;
        temperature_sum = 0;
    }
    /* USER CODE END ADC4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void show_number_on_seven_segment(int digit) {
    digits_off();
    int num = 0, vol_copy = volume;

    if (digit == 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
        num = vol_copy % 10;
    } else if (digit == 1) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
        vol_copy /= 10;
        num = vol_copy % 10;
    } else if (digit == 2) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 0);
        vol_copy /= 100;
        num = vol_copy % 10;
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, 0);
        vol_copy /= 1000;
        num = vol_copy % 10;
    }

    ic_input_value(num);
}

void digits_off() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 1);
}

void digits_on() {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 0);
    ic_input_value(8);
}

void ic_input_value(int num) {
    int bcd = BCD_Convertor(num);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (bcd % 10));
}

int BCD_Convertor(int number) {
    int converted[4];
    int bcd = 0;

    for (int i = 0; i < 4; i++) {
        converted[i] = number & (int) (pow(2, i));
        if (converted[i] > 0)
            converted[i] = 1;
        bcd += converted[i] * (pow(10, i));
    }

    return bcd;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
