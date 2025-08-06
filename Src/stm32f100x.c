/*
 * stm32f100x.c
 *
 *  Created on: Aug 6, 2025
 *      Author: osmannevfelunlu
 */

#include "stm32f100xx.h"

void RCC_EnableClock(void) {
  RCC->AHBENR  |= (1 << 0);  // Enable DMA1 Clock
  RCC->APB2ENR |= (1 << 2);  // Enable GPIOA Clock
  RCC->APB2ENR |= (1 << 3);  // Enable GPIOB Clock
  RCC->APB2ENR |= (1 << 4);  // Enable GPIOC Clock
  RCC->APB2ENR |= (1 << 5);  // Enable GPIOD Clock
  RCC->APB2ENR |= (1 << 6);  // Enable GPIOE Clock
  RCC->APB2ENR |= (1 << 14); // Enable USART1 Clock
  RCC->APB1ENR |= (1 << 0);  // Enable TIM2 Clock
  RCC->APB1ENR |= (1 << 1);  // Enable TIM3 Clock
  RCC->APB1ENR |= (1 << 2);  // Enable TIM4 Clock
  RCC->APB1ENR |= (1 << 17); // Enable USART2 Clock
  RCC->APB1ENR |= (1 << 18); // Enable USART3 Clock
}

uint64_t GPIOx[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};
enum GPIO {
	GPIO_A, GPIO_B, GPIO_C, GPIO_D, GPIO_E
};

void GPIO_SetPinOutput2(enum GPIO, uint8_t pin){
	GPIOx[GPIO]->CRL &= ~(0xF << (pin * 4));
	GPIOx[GPIO]->CRL |= (0x3 << (pin * 4));
}

void GPIOA_SetPinOutput(uint8_t pin) {
  GPIOA->CRL &= ~(0xF << (pin * 4));
  GPIOA->CRL |= (0x3 << (pin * 4));
}

void GPIOA_SetPinHigh(uint8_t pin) {
  GPIOA->BSRR = (1 << pin);
}

void GPIOA_SetPinLow(uint8_t pin) {
  GPIOA->BRR = (1 << pin);
}

void USART1_Init(uint32_t baudrate) {
  RCC->APB2ENR |= (1 << 14);
  USART1->BRR = 72000000 / baudrate;
  USART1->CR1 = (1 << 13) | (1 << 3) | (1 << 2);
}

void USART1_SendChar(char c) {
  while (!(USART1->SR & (1 << 7)));
  USART1->DR = c;
}

void USART1_SendString(char *str) {
  while (*str) USART1_SendChar(*str++);
}

void TIM2_Init(uint16_t prescaler, uint16_t arr) {
  RCC->APB1ENR |= (1 << 0);
  TIM2->PSC = prescaler - 1;
  TIM2->ARR = arr - 1;
  TIM2->CR1 |= (1 << 0);
}

void Delay_ms(uint32_t ms) {
  TIM2->CNT = 0;
  while (TIM2->CNT < ms);
}

void ADC1_Init(void) {
  RCC->APB2ENR |= (1 << 9);
  ADC1->CR2 |= (1 << 0);
  ADC1->SMPR2 |= (7 << 0);
}

uint16_t ADC1_Read(void) {
  ADC1->CR2 |= (1 << 22);
  while (!(ADC1->SR & (1 << 1)));
  return (uint16_t)ADC1->DR;
}
