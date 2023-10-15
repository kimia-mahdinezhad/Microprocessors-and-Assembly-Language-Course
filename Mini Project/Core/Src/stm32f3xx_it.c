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
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define password_not_set 0
#define waiting_for_entering_password 1
#define password_correct 0
#define password_incorrect 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int digit_value_1 = 0, digit_value_2 = 0, digit_value_3 = 0, digit_value_4 = 0;
int led_state = 1, buzzer_state = 0, buzzer_count = 0, seven_segment_state = 0;
int incorrect_password_attempt = 0;
int led_brightness = 0;
extern int program_state;
extern int password;
extern char buffer[4];
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

extern TIM_HandleTypeDef *pwm_timer; // Point to PWM timer configured in CubeMX
extern uint32_t pwm_channel;  // Specify configured PWM channel
void PWM_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
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
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
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
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void) {
    /* USER CODE BEGIN EXTI0_IRQn 0 */

    /* USER CODE END EXTI0_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    /* USER CODE BEGIN EXTI0_IRQn 1 */
    int entered_number = digit_value_1 + digit_value_2 * 10 + digit_value_3 * 100 + digit_value_4 * 1000;

    char entered_number_str[5];
    entered_number_str[0] = digit_value_4 + '0';
    entered_number_str[1] = digit_value_3 + '0';
    entered_number_str[2] = digit_value_2 + '0';
    entered_number_str[3] = digit_value_1 + '0';
    entered_number_str[4] = '\0';

    if (program_state != password_not_set) {
        if (password == entered_number) {
            program_state = password_correct;

            char message[36] = "Entered Password (";
            strcat(message, entered_number_str);
            strcat(message, ") is Correct.\n");
            HAL_UART_Transmit(&huart2, message, sizeof(message), 1000);

            HAL_NVIC_SystemReset();
        } else {
            program_state = password_incorrect;

            incorrect_password_attempt++;
            char incorrect_password_attempt_str[2];
            incorrect_password_attempt_str[0] = incorrect_password_attempt + '0';
            incorrect_password_attempt_str[1] = '\0';

            char message[70] = "Entered Password (";
            strcat(message, entered_number_str);
            strcat(message, ") is Incorrect.\nIncorrect Password Attempt = ");
            strcat(message, incorrect_password_attempt_str);
            strcat(message, ".\n");
            HAL_UART_Transmit(&huart2, message, sizeof(message), 1000);

            led_state = 1;
            buzzer_state = 1;
        }
    }
    /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void) {
    /* USER CODE BEGIN EXTI1_IRQn 0 */

    /* USER CODE END EXTI1_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    /* USER CODE BEGIN EXTI1_IRQn 1 */
    digit_value_1++;
    digit_value_1 %= 10;
    /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 and Touch Sense controller.
  */
void EXTI2_TSC_IRQHandler(void) {
    /* USER CODE BEGIN EXTI2_TSC_IRQn 0 */

    /* USER CODE END EXTI2_TSC_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    /* USER CODE BEGIN EXTI2_TSC_IRQn 1 */
    digit_value_2++;
    digit_value_2 %= 10;
    /* USER CODE END EXTI2_TSC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void) {
    /* USER CODE BEGIN EXTI3_IRQn 0 */

    /* USER CODE END EXTI3_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    /* USER CODE BEGIN EXTI3_IRQn 1 */
    digit_value_3++;
    digit_value_3 %= 10;
    /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void) {
    /* USER CODE BEGIN EXTI4_IRQn 0 */

    /* USER CODE END EXTI4_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    /* USER CODE BEGIN EXTI4_IRQn 1 */
    digit_value_4++;
    digit_value_4 %= 10;
    /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void) {
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    digits_off();
    if (program_state == waiting_for_entering_password) {
        show_number_on_seven_segment(seven_segment_state);
        seven_segment_state++;
        seven_segment_state %= 4;
    }
    /* USER CODE END TIM3_IRQn 1 */
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
  * @brief This function handles Timer 6 interrupt and DAC underrun interrupts.
  */
void TIM6_DAC_IRQHandler(void) {
    /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

    /* USER CODE END TIM6_DAC_IRQn 0 */
    HAL_TIM_IRQHandler(&htim6);
    /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

    if (incorrect_password_attempt == 1 && buzzer_count % 10 == 0) {
        PWM_Change_Tone(100, 100);
        PWM_Change_Tone(100, 0);
    } else if (incorrect_password_attempt == 2 && buzzer_count % 5 == 0) {
        PWM_Change_Tone(100, 100);
        PWM_Change_Tone(100, 0);
    } else if (incorrect_password_attempt >= 3) {
        PWM_Change_Tone(100, 100);
        PWM_Change_Tone(100, 0);
    }

    buzzer_count++;
    /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void) {
    /* USER CODE BEGIN TIM7_IRQn 0 */

    /* USER CODE END TIM7_IRQn 0 */
    HAL_TIM_IRQHandler(&htim7);
    /* USER CODE BEGIN TIM7_IRQn 1 */

    if (program_state == password_incorrect && led_state < 7) {
        if (led_state % 2 == 1) {
            led_brightness++;
            digits_on();

            if (led_brightness == 100) {
                led_state++;
                digits_off();
            }
        } else {
            led_brightness--;

            if (led_brightness == 0) {
                led_state++;
            }
        }

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, led_brightness);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, led_brightness);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, led_brightness);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, led_brightness);
    } else if (led_state >= 7) {
        program_state = waiting_for_entering_password;

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void show_number_on_seven_segment(int digit) {
    digits_off();
    int num = 0;

    if (digit == 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
        num = digit_value_1;
    } else if (digit == 1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
        num = digit_value_2;
    } else if (digit == 2) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
        num = digit_value_3;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
        num = digit_value_4;
    }

    ic_input_value(num);
}

void digits_off() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 1);
}

void digits_on() {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 0);
    ic_input_value(8);
}

void ic_input_value(int num) {
    int bcd = BCD_Convertor(num);

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (bcd % 10));
    bcd /= 10;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (bcd % 10));
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
