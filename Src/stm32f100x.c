/*
 * stm32f100x.c
 *
 *  Created on: Aug 6, 2025
 *      Author: osmannevfelunlu
 */

#include "stm32f100xx.h"

GPIO_TypeDef *GPIOx[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};
USART_TypeDef *USARTx[] = {USART1, USART2, USART3};
TIM_TypeDef *TIMx[] = {TIM1, TIM2, TIM3, TIM4};

void RCC_GPIO_Enable(void){
  RCC->APB2ENR |= (1 << 2);  // Enable GPIOA Clock
  RCC->APB2ENR |= (1 << 3);  // Enable GPIOB Clock
  RCC->APB2ENR |= (1 << 4);  // Enable GPIOC Clock
  RCC->APB2ENR |= (1 << 5);  // Enable GPIOD Clock
  RCC->APB2ENR |= (1 << 6);  // Enable GPIOE Clock
}

void RCC_USART_Enable(void){
  RCC->APB2ENR |= (1 << 14); // Enable USART1 Clock
  RCC->APB1ENR |= (1 << 17); // Enable USART2 Clock
  RCC->APB1ENR |= (1 << 18); // Enable USART3 Clock
}

void RCC_TIM_Enable(void){
  RCC->APB1ENR |= (1 << 0);  // Enable TIM2 Clock
  RCC->APB1ENR |= (1 << 1);  // Enable TIM3 Clock
  RCC->APB1ENR |= (1 << 2);  // Enable TIM4 Clock
}

void RCC_DMA_Enable(void) {
  RCC->AHBENR  |= (1 << 0);  // Enable DMA1 Clock
}

void GPIO_SetPinOutput(GPIO_Port port, uint8_t pin) {
	if(pin < 8){
		GPIOx[port]->CRL &= ~(0xF << (pin * 4));
		GPIOx[port]->CRL |= (0x3 << (pin * 4));
	} else if(pin >= 8 && pin <=15){
		GPIOx[port]->CRH &= ~(0xF << (pin % 8 * 4));
		GPIOx[port]->CRH |= (0x3 << (pin % 8 * 4));
	}
}

/*
void GPIOA_SetPinOutput(uint8_t pin) {
  GPIOA->CRL &= ~(0xF << (pin * 4));
  GPIOA->CRL |= (0x3 << (pin * 4));
}
*/
void GPIO_SetPinHigh(GPIO_Port port, uint8_t pin) {
    GPIOx[port]->BSRR = (1 << pin);
}
/*
void GPIOA_SetPinHigh(uint8_t pin) {
  GPIOA->BSRR = (1 << pin);
}
*/

/*
void GPIOA_SetPinLow(uint8_t pin) {
  GPIOA->BRR = (1 << pin);
}
*/
void GPIO_SetPinLow(GPIO_Port port,uint8_t pin) {
    GPIOx[port]->BRR = (1 << pin);
}

void USART_Init(USART_Port port, uint32_t baudrate) {
  RCC_USART_Enable();
  USARTx[port]->BRR = 72000000 / baudrate;
  USARTx[port]->CR1 = (1 << 13) | (1 << 3) | (1 << 2);
}
void USART_SendChar(USART_Port port, char c) {
  while (!(USARTx[port]->SR & (1 << 7)));
  USARTx[port]->DR = (uint8_t)c;
}

void USART_SendString(USART_Port port, char *str) {
  while (*str) USART_SendChar(port, *str++);
}
/*
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
*/

/*
void TIM2_Init(uint16_t prescaler, uint16_t arr) {
  RCC->APB1ENR |= (1 << 0);
  TIM2->PSC = prescaler - 1;
  TIM2->ARR = arr - 1;
  TIM2->CR1 |= (1 << 0);
}
*/
void TIM_Init(TIM_Port port, uint16_t prescaler, uint16_t arr) {
  RCC_TIM_Enable();
  TIMx[port]->PSC = prescaler - 1;
  TIMx[port]->ARR = arr - 1;
  TIMx[port]->CR1 |= (1 << 0);
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
