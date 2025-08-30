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

#define I2C_WRITE 0
#define I2C_READ  1

// I2C_CR1 bits
#define I2C_CR1_PE        (1U << 0)
#define I2C_CR1_START     (1U << 8)
#define I2C_CR1_STOP      (1U << 9)
#define I2C_CR1_ACK       (1U << 10)
#define I2C_CR1_SWRST     (1U << 15)

// I2C_SR1 bits
#define I2C_SR1_SB        (1U << 0)
#define I2C_SR1_ADDR      (1U << 1)
#define I2C_SR1_BTF       (1U << 2)
#define I2C_SR1_RXNE      (1U << 6)
#define I2C_SR1_TXE       (1U << 7)
#define I2C_SR1_BERR      (1U << 8)
#define I2C_SR1_ARLO      (1U << 9)
#define I2C_SR1_AF        (1U << 10)  // Acknowledge failure (NACK)
#define I2C_SR1_OVR       (1U << 11)
#define I2C_SR1_TIMEOUT   (1U << 14)


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
#define DMA1_BASE         (AHB_BASE + 0x0000UL)   /* DMA1 Controller */
#define DMA2_BASE         (AHB_BASE + 0x0400UL)   /* DMA2 Controller */
#define FLASH_R_BASE      (AHB_BASE + 0x2000UL)   /* Flash Registers */
#define CRC_BASE		  (AHB_BASE + 0x3000UL)   /* CRC */
/*** APB2 Peripherals ***/
#define GPIOA_BASE        (APB2_BASE + 0x0800UL)  /* GPIO Port A */
#define GPIOB_BASE        (APB2_BASE + 0x0C00UL)  /* GPIO Port B */
#define GPIOC_BASE        (APB2_BASE + 0x1000UL)  /* GPIO Port C */
#define GPIOD_BASE        (APB2_BASE + 0x1400UL)  /* GPIO Port D */
#define GPIOE_BASE        (APB2_BASE + 0x1800UL)  /* GPIO Port E */
#define GPIOF_BASE        (APB2_BASE + 0x1C00UL)  /* GPIO Port F */
#define GPIOG_BASE        (APB2_BASE + 0x2000UL)  /* GPIO Port G */
#define AFIO_BASE         (APB2_BASE + 0x0000UL)  /* Alternate Function IO */
#define EXTI_BASE         (APB2_BASE + 0x0400UL)  /* External Interrupt Controller */
#define ADC1_BASE		  (APB2_BASE + 0x2400UL)  /* ADC1 */
#define TIM1_BASE		  (APB2_BASE + 0x2C00UL)  /* Timer 1 */
#define USART1_BASE       (APB2_BASE + 0x3800UL)  /* USART1 */
#define SPI1_BASE		  (APB2_BASE + 0x3000UL)  /* SPI 1 */
#define TIM15_BASE        (APB2_BASE + 0x4000UL)  /* Timer 15 */
#define TIM16_BASE        (APB2_BASE + 0x4400UL)  /* Timer 16 */
#define TIM17_BASE        (APB2_BASE + 0x4800UL)  /* Timer 17 */

/*** APB1 Peripherals ***/
#define TIM2_BASE         (APB1_BASE + 0x0000UL)  /* Timer 2 */
#define TIM3_BASE         (APB1_BASE + 0x0400UL)  /* Timer 3 */
#define TIM4_BASE         (APB1_BASE + 0x0800UL)  /* Timer 4 */
#define TIM5_BASE         (APB1_BASE + 0x0C00UL)  /* Timer 5 */
#define TIM6_BASE         (APB1_BASE + 0x1000UL)  /* Timer 6 */
#define TIM7_BASE         (APB1_BASE + 0x1400UL)  /* Timer 7 */
#define TIM12_BASE        (APB1_BASE + 0x1800UL)  /* Timer 12 */
#define TIM13_BASE        (APB1_BASE + 0x1C00UL)  /* Timer 13 */
#define TIM14_BASE        (APB1_BASE + 0x2000UL)  /* Timer 14 */

#define RTC_BASE		  (APB1_BASE + 0x2800ul)  /* RTC */

#define SPI2_BASE		  (APB1_BASE + 0x3800UL)  /* SPI 2 */
#define SPI3_BASE		  (APB1_BASE + 0x3C00UL)  /* SPI 3 */

#define USART2_BASE       (APB1_BASE + 0x4400UL)  /* USART2 */
#define USART3_BASE       (APB1_BASE + 0x4800UL)  /* USART3 */

#define I2C1_BASE		  (APB1_BASE + 0x5400UL)  /* I2C 1 */
#define I2C2_BASE		  (APB1_BASE + 0x5800UL)  /* I2C 2 */


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

/* I2C Register Structure */
typedef struct {
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t OAR1;
  __IO uint32_t OAR2;
  __IO uint32_t DR;
  __IO uint32_t SR1;
  __IO uint32_t SR2;
  __IO uint32_t CCR;
  __IO uint32_t TRISE;
} I2C_TypeDef;

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

/* SPI Register Structure */
typedef struct {
  __IO uint32_t CR1;
  __IO uint32_t CR2;
  __IO uint32_t SR;
  __IO uint32_t DR;
  __IO uint32_t CRCPR;
  __IO uint32_t RXCRCR;
  __IO uint32_t TXCRCR;
} SPI_TypeDef;

/* CRC Register Structure */
typedef struct {
  __IO uint32_t DR;
  __IO uint32_t IDR;
  __IO uint32_t CR;
} CRC_TypeDef;

/* RTC Register Structure */
typedef struct {
  __IO uint32_t CRH;
  __IO uint32_t CRL;
  __IO uint32_t PRLH;
  __IO uint32_t PRLL;
  __IO uint32_t DIVH;
  __IO uint32_t DIVL;
  __IO uint32_t CNTH;
  __IO uint32_t CNTL;
  __IO uint32_t ALRH;
  __IO uint32_t ALRL;
} RTC_TypeDef;

/* DMA Register Structure */
typedef struct {
  __IO uint32_t ISR;
  __IO uint32_t IFCR;
  __IO uint32_t CCR1;
  __IO uint32_t CNDTR1;
  __IO uint32_t CPAR1;
  __IO uint32_t CMAR1;
  __IO uint32_t CCR2;
  __IO uint32_t CNDTR2;
  __IO uint32_t CPAR2;
  __IO uint32_t CMAR2;
  __IO uint32_t CCR3;
  __IO uint32_t CNDTR3;
  __IO uint32_t CPAR3;
  __IO uint32_t CMAR3;
  __IO uint32_t CCR4;
  __IO uint32_t CNDTR4;
  __IO uint32_t CPAR4;
  __IO uint32_t CMAR4;
  __IO uint32_t CCR5;
  __IO uint32_t CNDTR5;
  __IO uint32_t CPAR5;
  __IO uint32_t CMAR5;
  __IO uint32_t CCR6;
  __IO uint32_t CNDTR6;
  __IO uint32_t CPAR6;
  __IO uint32_t CMAR6;
  __IO uint32_t CCR7;
  __IO uint32_t CNDTR7;
  __IO uint32_t CPAR7;
  __IO uint32_t CMAR7;
} DMA_TypeDef;

typedef enum {
  I2C_OK = 0,
  I2C_ERROR_TIMEOUT,
  I2C_ERROR_NACK,
  I2C_ERROR_BUS,       // bus error
  I2C_ERROR_ARB_LOST   // arbitration lost
} I2C_Status;

#define RCC    ((RCC_TypeDef *) RCC_BASE)

#define GPIOA  ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB  ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC  ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD  ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE  ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF  ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG  ((GPIO_TypeDef *) GPIOG_BASE)

#define USART1 ((USART_TypeDef *) USART1_BASE)
#define USART2 ((USART_TypeDef *) USART2_BASE)
#define USART3 ((USART_TypeDef *) USART3_BASE)

#define ADC1   ((ADC_TypeDef *) ADC1_BASE)

#define TIM1   ((TIM_TypeDef *) TIM1_BASE)
#define TIM2   ((TIM_TypeDef *) TIM2_BASE)
#define TIM3   ((TIM_TypeDef *) TIM3_BASE)
#define TIM4   ((TIM_TypeDef *) TIM4_BASE)
#define TIM5   ((TIM_TypeDef *) TIM5_BASE)
#define TIM6   ((TIM_TypeDef *) TIM6_BASE)
#define TIM7   ((TIM_TypeDef *) TIM7_BASE)
#define TIM12  ((TIM_TypeDef *) TIM12_BASE)
#define TIM13  ((TIM_TypeDef *) TIM13_BASE)
#define TIM14  ((TIM_TypeDef *) TIM14_BASE)
#define TIM15  ((TIM_TypeDef *) TIM15_BASE)
#define TIM16  ((TIM_TypeDef *) TIM16_BASE)
#define TIM17  ((TIM_TypeDef *) TIM17_BASE)

#define I2C1   ((I2C_TypeDef *) I2C1_BASE)
#define I2C2   ((I2C_TypeDef *) I2C2_BASE)

#define SPI1   ((SPI_TypeDef *) SPI1_BASE)
#define SPI2   ((SPI_TypeDef *) SPI2_BASE)
#define SPI3   ((SPI_TypeDef *) SPI3_BASE)

#define CRC	   ((CRC_TypeDef *) CRC_BASE)

#define DMA1   ((DMA_TypeDef *) DMA1_BASE)
#define DMA2   ((DMA_TypeDef *) DMA2_BASE)

#define RTC	   ((RTC_TypeDef *) RTC_BASE)

typedef enum {
    GPIO_A, GPIO_B, GPIO_C, GPIO_D, GPIO_E, GPIO_F, GPIO_G
} GPIO_Port;

typedef enum {
	USART_1, USART_2, USART_3
} USART_Port;

typedef enum {
	TIM_2, TIM_3, TIM_4, TIM_6, TIM_7, TIM_15, TIM_16, TIM_17
} TIM_Port;

typedef enum {
	I2C_1, I2C_2
} I2C_Port;

typedef enum {
	SPI_1, SPI_2
} SPI_Port;

typedef enum {
	DMA_1, DMA_2
} DMA_Port;

/* Function Prototypes */
void RCC_GPIO_Enable(void);
void RCC_USART_Enable(void);
void RCC_TIM_Enable(void);
void RCC_DMA_Enable(void);
void RCC_ADC_Enable(void);
void RCC_I2C_Enable(void);

void GPIO_SetPinHigh(GPIO_Port port, uint8_t pin);
void GPIO_SetPinOutput(GPIO_Port port, uint8_t pin);
void GPIO_SetPinLow(GPIO_Port port,uint8_t pin);
uint8_t GPIO_ReadPin(GPIO_Port port, uint8_t pin);

void I2C_Init(I2C_Port port);
void SPI_Init(SPI_Port port);
void USART_Init(USART_Port port, uint32_t baudrate);
void USART_SendChar(USART_Port port, char c);
void USART_SendString(USART_Port port, char *str);

void ADC_Init(void);
void TIM_Init(TIM_Port port, uint16_t prescaler, uint16_t arr);
void Delay_ms(uint32_t ms);
uint16_t ADC1_Read(void);

I2C_Status I2C_Start(I2C_Port port, uint8_t slave_addr_7bit, uint8_t direction, uint32_t timeout_ms);
I2C_Status I2C_WriteByte(I2C_Port port, uint8_t data, uint32_t timeout_ms);
I2C_Status I2C_ReadByte(I2C_Port port, uint8_t *out_data, uint8_t ack, uint32_t timeout_ms);
void I2C_Stop(I2C_Port port);

static inline I2C_TypeDef* I2C_GetPort(I2C_Port port);
/*
 * void RCC_EnableClock(void);
 * void GPIOA_SetPinOutput(uint8_t pin);
 * void GPIOA_SetPinHigh(uint8_t pin);
 * void USART1_Init(uint32_t baudrate);
 * void USART1_SendChar(char c);
 * void USART1_SendString(char *str);
 * void TIM2_Init(uint16_t prescaler, uint16_t arr);
 * void GPIOA_SetPinLow(uint8_t pin);
 *
 */
#endif /* STM32F100XX_H_ */
