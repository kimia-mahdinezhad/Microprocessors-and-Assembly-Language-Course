#include "stm32f3xx_hal.h"
#include "math.h"

#define digit GPIOB
#define dot GPIO_PIN_11
#define d1 GPIO_PIN_12
#define d2 GPIO_PIN_13
#define d3 GPIO_PIN_14
#define d4 GPIO_PIN_15

#define ic GPIOA
#define a GPIO_PIN_1
#define b GPIO_PIN_2
#define c GPIO_PIN_3
#define d GPIO_PIN_4

int BCD_Convertor(int);

void ic_input_init();
void seven_segment_digit_init();
void button_init();

int calculate_count_unit();
void count(int);

void show_4_digit_number(int, int);
void ic_input_value(int);
void digits_off();

int button_state = 0;
int debounce_time = 0;


int main() {
	HAL_SYSTICK_Config(SystemCoreClock / (1000U / 1));

	ic_input_init();
	seven_segment_digit_init();
	button_init();

	while (1) {
		int count_unit = calculate_count_unit();
		button_state = 0;
		count(count_unit);
	}
}

int calculate_count_unit () {
	int count_unit = 0;

	while (!button_state) {
		count_unit++;
				count_unit %= 10;
		for (int passed_time = 0 ; passed_time <= 500 ;) {
			int start = HAL_GetTick();
			show_4_digit_number(count_unit * 1000, 0);
			passed_time += HAL_GetTick() - start;
		}
	}

	if (count_unit == 0)
		count_unit = 10;

	return count_unit;
}

void count (int count_unit) {
	int counter = 0;

	while (1) {
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) && HAL_GetTick() - debounce_time >= 2000)
			HAL_NVIC_SystemReset();

		for (int passed_time = 0 ; passed_time <= count_unit * 100 ;) {
			int start = HAL_GetTick();
			show_4_digit_number(count_unit * 1000 + counter, 1);
			passed_time += HAL_GetTick() - start;
		}

		if (button_state != 1) {
			counter += count_unit;

			if (counter >= 999)
				HAL_NVIC_SystemReset();
		}
	}
}

void show_4_digit_number (int num, int dot_checked) {
	HAL_GPIO_WritePin(digit, d4, 0);
	ic_input_value(num % 10);
	HAL_Delay(4);
	digits_off();

	num /= 10;

	if (dot_checked)
		HAL_GPIO_WritePin(digit, dot, 1);

	HAL_GPIO_WritePin(digit, d3, 0);
	ic_input_value(num % 10);
	HAL_Delay(4);
	digits_off();

	num /= 10;

	HAL_GPIO_WritePin(digit, d2, 0);
	ic_input_value(num % 10);
	HAL_Delay(4);
	digits_off();

	num /= 10;

	HAL_GPIO_WritePin(digit, d1, 0);
	ic_input_value(num % 10);
	HAL_Delay(4);
	digits_off();
}

void digits_off () {
	HAL_GPIO_WritePin(digit, d1 | d2 | d3 | d4, 1);
	HAL_GPIO_WritePin(digit, dot, 0);
	HAL_GPIO_WritePin(ic, a | b | c | d, 1);
}

void ic_input_value (int num) {
	int bcd = BCD_Convertor(num);

	HAL_GPIO_WritePin(ic, a, (bcd % 10));

	bcd /= 10;

	HAL_GPIO_WritePin(ic, b, (bcd % 10));

	bcd /= 10;

	HAL_GPIO_WritePin(ic, c, (bcd % 10));

	bcd /= 10;

	HAL_GPIO_WritePin(ic, d, (bcd % 10));
}

int BCD_Convertor (int number) {
	int converted[4];
	int bcd = 0;

	for (int i = 0 ; i < 4 ; i++) {
		converted[i] = number & (int) (pow(2, i));
		if (converted[i] > 0)
			converted[i] = 1;
		bcd += converted[i] * (pow(10, i));
	}

	return bcd;
}

void EXTI0_IRQHandler (void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

	if (button_state == 1)
		button_state = 0;
	else {
		button_state = 1;
		debounce_time = HAL_GetTick();
	}
}

void ic_input_init() {
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_PinOut;
	GPIO_PinOut.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
	GPIO_PinOut.Mode = GPIO_MODE_OUTPUT_PP;

	HAL_GPIO_Init(GPIOA, &GPIO_PinOut);
}

void seven_segment_digit_init() {
	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_PinOut;
	GPIO_PinOut.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_PinOut.Mode = GPIO_MODE_OUTPUT_PP;

	HAL_GPIO_Init(GPIOB, &GPIO_PinOut);
}

void button_init() {
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_PinOut;

	GPIO_PinOut.Pin = GPIO_PIN_0;
	GPIO_PinOut.Mode = GPIO_MODE_IT_RISING;
	GPIO_PinOut.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_PinOut.Pull = GPIO_PULLDOWN;

	HAL_GPIO_Init(GPIOA, &GPIO_PinOut);

	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

