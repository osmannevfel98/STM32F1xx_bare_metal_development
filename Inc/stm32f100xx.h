/*
 * stm32f100xx.h
 *
 *  Created on: Aug 6, 2025
 *      Author: osmannevfelunlu
 */

#ifndef STM32F100XX_H_
#define STM32F100XX_H_

#include <stdint.h>

#define __IO volatile

/*** Memory Base Addresses ***/
#define FLASH_PROG_BASE   (0x0008000UL)  /* Flash Program Memory Base Address */
#define SRAM_BASE         (0x20000000UL) /* SRAM Base Address */

/*** Peripheral Base Addresses ***/
#define PERIPH_BASE       (0x4000000UL)  /* Base Address for peripherals */
#define APB1_BASE         PERIPH_BASE    /* APB1 Bus Base Address */
#define APB2_BASE         (PERIPH_BASE + 0x10000UL)  /* APB2 Bus Base Address */
#define AHB_BASE          (PERIPH_BASE + 0x20000UL)  /* AHB Bus Base Address */

/*** AHB Peripherals ***/
#define RCC_BASE          (AHB_BASE + 0x1000UL)   /* Reset and Clock Control */
#define DMA1_BASE         (AHB_BASE + 0x8000UL)   /* DMA1 Controller */
#define FLASH_R_BASE      (AHB_BASE + 0x2000UL)   /* Flash Registers */

/*** APB2 Peripherals ***/
#define GPIOA_BASE        (APB2_BASE + 0x0800UL)  /* GPIO Port A */
#define GPIOB_BASE        (APB2_BASE + 0x0C00UL)  /* GPIO Port B */
#define GPIOC_BASE        (APB2_BASE + 0x1000UL)  /* GPIO Port C */
#define GPIOD_BASE        (APB2_BASE + 0x1400UL)  /* GPIO Port D */
#define GPIOE_BASE        (APB2_BASE + 0x1800UL)  /* GPIO Port E */
#define AFIO_BASE         (APB2_BASE + 0x0000UL)  /* Alternate Function IO */
#define EXTI_BASE         (APB2_BASE + 0x0400UL)  /* External Interrupt Controller */

/*** APB1 Peripherals ***/
#define USART1_BASE       (APB2_BASE + 0x3800UL)  /* USART1 */
#define USART2_BASE       (APB1_BASE + 0x4400UL)  /* USART2 */
#define USART3_BASE       (APB1_BASE + 0x4800UL)  /* USART3 */
#define TIM2_BASE         (APB1_BASE + 0x0000UL)  /* Timer 2 */
#define TIM3_BASE         (APB1_BASE + 0x0400UL)  /* Timer 3 */
#define TIM4_BASE         (APB1_BASE + 0x0800UL)  /* Timer 4 */

/* RCC Register Structure */
typedef struct {
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
} RCC_TypeDef;

#define RCC  ((RCC_TypeDef *) RCC_BASE)

/* GPIO Register Structure */
typedef struct {
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

#define GPIOA  ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD  ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE  ((GPIO_TypeDef *) GPIOE_BASE)

/* USART Register Structure */
typedef struct {
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t BRR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t CR3;
  __IO uint32_t GTPR;
} USART_TypeDef;

#define USART1  ((USART_TypeDef *) USART1_BASE)
#define USART2  ((USART_TypeDef *) USART2_BASE)
#define USART3  ((USART_TypeDef *) USART3_BASE)

/* Timer Register Structure */
typedef struct {
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMCR;
  __IO uint32_t DIER;
  __IO uint32_t SR;
  __IO uint32_t EGR;
  __IO uint32_t CCMR1;
  __IO uint32_t CCMR2;
  __IO uint32_t CCER;
  __IO uint32_t CNT;
  __IO uint32_t PSC;
  __IO uint32_t ARR;
} TIM_TypeDef;

#define TIM2  ((TIM_TypeDef *) TIM2_BASE)
#define TIM3  ((TIM_TypeDef *) TIM3_BASE)
#define TIM4  ((TIM_TypeDef *) TIM4_BASE)

/* ADC Register Structure */
typedef struct {
  __IO uint32_t SR;
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SMPR1;
  __IO uint32_t SMPR2;
  __IO uint32_t JOFR1;
  __IO uint32_t JOFR2;
  __IO uint32_t JOFR3;
  __IO uint32_t JOFR4;
  __IO uint32_t HTR;
  __IO uint32_t LTR;
  __IO uint32_t SQR1;
  __IO uint32_t SQR2;
  __IO uint32_t SQR3;
  __IO uint32_t JSQR;
  __IO uint32_t JDR1;
  __IO uint32_t JDR2;
  __IO uint32_t JDR3;
  __IO uint32_t JDR4;
  __IO uint32_t DR;
} ADC_TypeDef;

#define ADC1  ((ADC_TypeDef *) 0x40012400UL) /* ADC1 Base Address */

typedef enum {
    GPIO_A, GPIO_B, GPIO_C, GPIO_D, GPIO_E
} GPIO_Port;

/* Function Prototypes */
void RCC_EnableClock(void);
//void GPIOA_SetPinOutput(uint8_t pin);
//void GPIOA_SetPinHigh(uint8_t pin);
void GPIO_SetPinHigh(GPIO_Port port, uint8_t pin);
void GPIO_SetPinOutput(GPIO_Port port, uint8_t pin);
//void GPIOA_SetPinLow(uint8_t pin);
void GPIO_SetPinLow(GPIO_Port port,uint8_t pin);
void USART1_Init(uint32_t baudrate);
void USART1_SendChar(char c);
void USART1_SendString(char *str);
void TIM2_Init(uint16_t prescaler, uint16_t arr);
void Delay_ms(uint32_t ms);
void ADC1_Init(void);
uint16_t ADC1_Read(void);

#endif /* STM32F100XX_H_ */
