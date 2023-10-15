/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal.h"
#include "stdlib.h"
#include "math.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void set_screen();

void generate_enemy();

void move_objects();

void show_objects();

void show_player();

int check_for_losing();

void show_number_on_seven_segment(int);

void digits_off();

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
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */
int button_state = 0, jump_state = -1, seven_segment_state = 0, message_state = 0;
int enemy_up_position[20] = {0};
int enemy_down_position[20] = {0};
int player_position = 2;
int score = 0;
int message_counter = 0;

extern byte player[];
extern byte enemy_down[];
extern byte enemy_up[];
extern byte ground[];
extern byte blank[];
extern byte letter_1_message_1[];
extern byte letter_2_message_1[];
extern byte letter_4_message_1[];
extern byte letter_5_message_1[];
extern byte letter_6_message_1[];
extern byte letter_1_message_2[];
extern byte letter_2_message_2[];
extern byte letter_3_message_2[];
extern byte letter_2_message_3[];
extern byte letter_3_message_3[];
extern byte letter_4_message_3[];
extern byte letter_5_message_3[];
extern byte letter_6_message_3[];
extern byte letter_1_message_4[];
extern byte letter_2_message_4[];
extern byte letter_3_message_4[];
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
    if (button_state == 0) {
        begin(20, 4);
        set_screen();
        button_state = 1;
    }

    if (jump_state == 0) {
        jump_state = 1;
        player_position = 0;
        show_player();
        PWM_Change_Tone(500, 100);
    }
    /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void) {
    /* USER CODE BEGIN TIM3_IRQn 0 */

    /* USER CODE END TIM3_IRQn 0 */
    HAL_TIM_IRQHandler(&htim3);
    /* USER CODE BEGIN TIM3_IRQn 1 */
    if (message_state == 0) {
        if (message_counter % 2 == 0) {
            createChar(0, letter_1_message_1);
            createChar(1, letter_2_message_1);
            createChar(2, letter_4_message_1);
            createChar(3, letter_5_message_1);
            createChar(4, letter_6_message_1);

            setCursor(7, 2);

            write(4);
            write(3);
            write(2);
            write(0);
            write(1);
            write(0);
        } else {
            begin(20, 4);

            createChar(0, letter_1_message_2);
            createChar(1, letter_2_message_2);
            createChar(2, letter_3_message_2);

            setCursor(8, 0);

            write(2);
            write(1);
            write(0);

            setCursor(2, 2);
            print("Press BlueButton");

            message_state = 1;
        }
        message_counter++;
    } else if (message_state == 2) {
        if (message_counter % 2 == 0) {
            begin(20, 4);

            createChar(0, letter_1_message_2);
            createChar(1, letter_2_message_3);
            createChar(2, letter_3_message_3);
            createChar(3, letter_4_message_3);
            createChar(4, letter_5_message_3);
            createChar(5, letter_6_message_3);

            setCursor(7, 2);

            write(5);
            write(4);
            write(3);
            write(2);
            write(1);
            write(0);
        } else {
            begin(20, 4);

            createChar(0, letter_1_message_4);
            createChar(1, letter_2_message_4);
            createChar(2, letter_3_message_4);

            setCursor(10, 2);

            write(2);
            write(1);
            write(0);

            setCursor(8, 2);

            char score_str[5];
            itoa(score, score_str, 10);
            print(score_str);

            message_state = 3;
        }
        message_counter++;
    }

    if (button_state == 1) {
        if (jump_state == 1) {
            jump_state = 2;
            PWM_Change_Tone(500, 0);
        } else if (jump_state == 2) {
            player_position = 2;
            show_player();
            jump_state = 0;
        }

        generate_enemy();
        move_objects();

        score++;
    }
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

    if (button_state != 0) {
        show_number_on_seven_segment(seven_segment_state);
        seven_segment_state++;
        seven_segment_state %= 4;
    }
    /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void set_screen() {
    srand(HAL_GetTick());
    int random = 0;

    createChar(0, player);
    createChar(1, enemy_up);
    createChar(2, enemy_down);
    createChar(3, ground);
    createChar(4, blank);

    show_player();

    for (int i = 0; i < 20; ++i) {
        setCursor(i, 3);
        write(3);
    }

    for (int i = 8; i < 20; i++) {
        random = rand() % 100;

        if (random >= 0 && random <= 35) {
            if (enemy_down_position[i - 1] != 1) {
                enemy_up_position[i] = 1;
                setCursor(i, 0);
                write(1);
            }
        } else if (random >= 36 && random <= 80) {
            if (enemy_down_position[i - 1] != 1) {
                if (enemy_up_position[i - 1] != 1 && enemy_up_position[i] != 1) {
                    enemy_down_position[i] = 1;
                    setCursor(i, 2);
                    write(2);
                }
            }
        }
    }
}

void generate_enemy() {
    srand(HAL_GetTick());
    int random = 0;

    random = rand() % 100;

    if (random >= 0 && random <= 35) {
        if (enemy_down_position[18] != 1) {
            enemy_up_position[19] = 1;
        }
    } else if (random >= 36 && random <= 80) {
        if (enemy_down_position[18] != 1) {
            if (enemy_up_position[18] != 1 && enemy_up_position[19] != 1) {
                enemy_down_position[19] = 1;
            }
        }
    }
}

void move_objects() {
    show_objects();
    for (int i = 0; i < 19; i++) {
        enemy_up_position[i] = enemy_up_position[i + 1];
        enemy_down_position[i] = enemy_down_position[i + 1];
    }

    enemy_up_position[19] = 0;
    enemy_down_position[19] = 0;
}

void show_objects() {
    for (int i = 0; i < 20; i++) {
        if (check_for_losing()) {
            button_state = 2;
            message_state = 2;
            PWM_Change_Tone(500, 0);
            break;
        }
        if (i != player_position || (i == player_position && jump_state != 1)) {
            setCursor(i, 0);
            if (enemy_up_position[i] == 0) {
                write(4);
            } else {
                write(1);
            }
        }

        if (i != player_position || (i == player_position && jump_state == 1)) {
            setCursor(i, 2);
            if (enemy_down_position[i] == 0) {
                write(4);
            } else {
                write(2);
            }
        }
    }
}

void show_player() {
    setCursor(2, player_position);
    write(0);
    setCursor(2, 2 - player_position);
    write(4);
}

int check_for_losing() {
    return (enemy_up_position[2] == 1 && player_position == 0) || (enemy_down_position[2] == 1 && player_position == 2);
}

void show_number_on_seven_segment(int digit) {
    digits_off();
    int num = 0, score_copy = score;

    if (digit == 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
        num = score_copy % 10;
    } else if (digit == 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
        score_copy /= 10;
        num = score_copy % 10;
    } else if (digit == 2) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
        score_copy /= 100;
        num = score_copy % 10;
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
        score_copy /= 1000;
        num = score_copy % 10;
    }

    ic_input_value(num);
}

void digits_off() {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, 1);
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
