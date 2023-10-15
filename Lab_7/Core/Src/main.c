/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdio.h"
#include "math.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
GPIO_TypeDef *const Row_ports[] = {GPIOD, GPIOD, GPIOD, GPIOB};
const uint16_t Row_pins[] = {GPIO_PIN_3, GPIO_PIN_5, GPIO_PIN_7, GPIO_PIN_4};
GPIO_TypeDef *const Column_ports[] = {GPIOD, GPIOD, GPIOB, GPIOB};
const uint16_t Column_pins[] = {GPIO_PIN_4, GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_5};
volatile uint32_t last_gpio_exti;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int operator = 0, number_2 = 0;
float number_1 = 0, result = 0;
int get_number_state = 1;
int negative_flag = 0, error_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void manage_input(int);
int get_digit_or_operator(int);
float calculate_result();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (last_gpio_exti + 200 > HAL_GetTick()) {
        return;
    }
    last_gpio_exti = HAL_GetTick();

    int8_t row_number = -1;
    int8_t column_number = -1;

    for (uint8_t row = 0; row < 4; row++) {
        if (GPIO_Pin == Row_pins[row]) {
            row_number = row;
        }
    }

    HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 0);
    HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 0);
    HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 0);
    HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 0);

    for (uint8_t col = 0; col < 4; col++) {
        HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 1);
        if (HAL_GPIO_ReadPin(Row_ports[row_number], Row_pins[row_number])) {
            column_number = col;
        }
        HAL_GPIO_WritePin(Column_ports[col], Column_pins[col], 0);
    }

    HAL_GPIO_WritePin(Column_ports[0], Column_pins[0], 1);
    HAL_GPIO_WritePin(Column_ports[1], Column_pins[1], 1);
    HAL_GPIO_WritePin(Column_ports[2], Column_pins[2], 1);
    HAL_GPIO_WritePin(Column_ports[3], Column_pins[3], 1);

    if (row_number == -1 || column_number == -1) {
        return;
    }

    const uint8_t button_number = row_number * 4 + column_number + 1;

    manage_input(button_number);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    /* USER CODE BEGIN 2 */
    LiquidCrystal(GPIOD, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14);
    begin(20, 4);
    setCursor(1, 0);
    print("0");
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                             | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_SET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_SET);

    /*Configure GPIO pins : PD8 PD9 PD10 PD11
                             PD12 PD13 PD14 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                          | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PD3 PD5 PD7 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PD4 PD6 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB5 */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void manage_input (int button_number) {
    int digit_or_operator = get_digit_or_operator(button_number);

    /* Erase Zero */
    if (number_1 == 0 && negative_flag == 0 && error_flag == 0) {
        begin(20, 4);
        setCursor(1, 0);
    }

    /* Get First Number */
    if (get_number_state == 1 && number_1 < 1000 && digit_or_operator < 10) {
        number_1 = number_1 * 10 + digit_or_operator;
        if (negative_flag == 1) {
            number_1 *= -1;
            negative_flag = 0;
        }
        char show[2];
        show[0] = digit_or_operator + '0';
        show[1] = '\0';
        print(show);
    }

    /* Negative Number or Subtraction? */
    if (digit_or_operator == 45) {
        if (number_1 == 0) {
            negative_flag = 1;
            char show[2];
            show[0] = digit_or_operator;
            show[1] = '\0';
            print(show);
        } else {
            operator = digit_or_operator;
            setCursor(1, 1);
            char show[2];
            show[0] = digit_or_operator;
            show[1] = '\0';
            print(show);
            get_number_state = 2;
        }
    }

    /* Operations (Except for Subtraction) */
    if (digit_or_operator == 42 || digit_or_operator == 43 || digit_or_operator == 47) {
        operator = digit_or_operator;
        setCursor(1, 1);
        char show[2];
        show[0] = digit_or_operator;
        show[1] = '\0';
        print(show);
        get_number_state = 2;
    }

    /* Change Row for Print on LCD */
    if (get_number_state == 2 && number_2 == 0)
        setCursor(1, 2);

    /* Get Second Number */
    if (get_number_state == 2 && number_2 < 1000 && digit_or_operator < 10) {
        number_2 = number_2 * 10 + digit_or_operator;
        char show[2];
        show[0] = digit_or_operator + '0';
        show[1] = '\0';
        print(show);
    }

    /* Calculate Result */
    if (digit_or_operator == 61 && get_number_state != 1) {
        result = calculate_result();
        number_1 = result;
        char show[20];

        if (error_flag == 0) {
            if (result == sqrt(result * result) || -1 * result == sqrt(result * result)) {
                sprintf(show, "%.0f", result);
            } else {
                sprintf(show, "%.4f", result);
            }

            begin(20, 4);
            setCursor(1, 0);
            print(show);
            number_2 = 0;
            operator = 0;
            get_number_state = 0;
        }
    }

    /* Clear */
    if (digit_or_operator == 67) {
        begin(20, 4);
        setCursor(1, 0);
        print("0");
        number_1 = 0;
        number_2 = 0;
        operator = 0;
        get_number_state = 1;

        if (error_flag == 1) {
            error_flag = 0;
        }
    }

    /* Change state for only getting 1 to 4 digit numbers for the first number */
    if (get_number_state == 1 && ((number_1 >= 1000 && number_1 < 10000) || (number_1 > -10000 && number_1 <= -1000))) {
        get_number_state = 0;
        operator = 0;
    }

    /* Change state for operation */
    if (get_number_state == 0 && operator != 0) {
        get_number_state = 2;
        number_2 = 0;
    }

    /* Change state for only getting 1 to 4 digit numbers for the second number */
    if (get_number_state == 2 && ((number_2 >= 1000 && number_2 < 10000) || (number_2 > -10000 && number_2 <= -1000))) {
        get_number_state = 3;
    }
}

int get_digit_or_operator (int button_number) {
    if (button_number == 1 || button_number == 2 || button_number == 3)
        return button_number;
    else if (button_number == 4)
        return 45;
    else if (button_number == 5 ||button_number == 6 || button_number == 7)
        return button_number - 1;
    else if (button_number == 8)
        return 43;
    else if (button_number == 9 ||button_number == 10 || button_number == 11)
        return button_number - 2;
    else if (button_number == 12)
        return 42;
    else if (button_number == 13)
        return 67;
    else if (button_number == 14)
        return 0;
    else if (button_number == 15)
        return 61;
    else if (button_number == 16)
        return 47;
    else
        return -1;
}

float calculate_result () {
    if (operator == 45)
        return (float) (number_1 - number_2);
    else if (operator == 43)
        return (float) (number_1 + number_2);
    else if (operator == 42)
        return (float) (number_1 * number_2);
    else if (operator == 47) {
        if (number_2 != 0)
            return (float) number_1 / (float) number_2;
        else {
            begin(20, 4);
            setCursor(1, 0);
            print("Invalid Operation");
            setCursor(1, 1);
            print("Division by Zero!");
            setCursor(1, 3);
            print("Press C to Continue");
            number_1 = 0;
            number_2 = 0;
            operator = 0;
            get_number_state = 1;
            error_flag = 1;
            return 0;
        }
    }
    else
        return 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
