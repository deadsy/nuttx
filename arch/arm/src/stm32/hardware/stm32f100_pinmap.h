/************************************************************************************
 * arch/arm/src/stm32/hardware/stm32f100_pinmap.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2012 Michael Smith. All Rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Uros Platise <uros.platise@isotel.eu>
 *           Michael Smith
 *           Freddie Chopin <freddie_chopin@op.pl>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_HARDWARE_STM32F100_PINMAP_H
#define __ARCH_ARM_SRC_STM32_HARDWARE_STM32F100_PINMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "stm32_gpio.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Alternate Pin Functions: */

/* ADC */

#define GPIO_ADC1_IN0       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_ADC1_IN1       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#define GPIO_ADC1_IN2       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#define GPIO_ADC1_IN3       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#define GPIO_ADC1_IN4       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#define GPIO_ADC1_IN5       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)
#define GPIO_ADC1_IN6       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#define GPIO_ADC1_IN7       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#define GPIO_ADC1_IN8       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#define GPIO_ADC1_IN9       (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#define GPIO_ADC1_IN10      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN0)
#define GPIO_ADC1_IN11      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN1)
#define GPIO_ADC1_IN12      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN2)
#define GPIO_ADC1_IN13      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN3)
#define GPIO_ADC1_IN14      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN4)
#define GPIO_ADC1_IN15      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN5)

/* CEC */
#if defined(CONFIG_STM32_CEC_REMAP)
#  define GPIO_CEC          (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#else
#  define GPIO_CEC          (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#endif

/* DAC
 * Note from RM0041, 11.2: "Once the DAC channelx is enabled, the corresponding
 * GPIO pin (PA4 or PA5) is automatically connected to the analog converter
 * output (DAC_OUTx). In order to avoid parasitic consumption, the PA4 or PA5
 * pin should first be configured to analog (AIN)."
 */

#define GPIO_DAC1_OUT1      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN4)
#define GPIO_DAC1_OUT2      (GPIO_INPUT|GPIO_CNF_ANALOGIN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN5)

/* FSMC */

/* TODO - VL devices in 100- and 144-pin packages have FSMC */

/* I2C */

#if defined(CONFIG_STM32_I2C1_REMAP)
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#else
#  define GPIO_I2C1_SCL     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_I2C1_SDA     (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#endif
#define GPIO_I2C1_SMBA      (GPIO_ALT|GPIO_CNF_INFLOAT|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)

#define GPIO_I2C2_SCL       (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#define GPIO_I2C2_SDA       (GPIO_ALT|GPIO_CNF_AFOD|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#define GPIO_I2C2_SMBA      (GPIO_ALT|GPIO_CNF_INFLOAT|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)

/* SPI */

#if defined(CONFIG_STM32_SPI1_REMAP)
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#else
#  define GPIO_SPI1_NSS     (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#  define GPIO_SPI1_SCK     (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN5)
#  define GPIO_SPI1_MISO    (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_SPI1_MOSI    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#endif

#define GPIO_SPI2_NSS       (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SPI2_SCK       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#define GPIO_SPI2_MISO      (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SPI2_MOSI      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)

#define GPIO_SPI3_NSS       (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#define GPIO_SPI3_SCK       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#define GPIO_SPI3_MISO      (GPIO_INPUT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN4)
#define GPIO_SPI3_MOSI      (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)

/* TIMERS */

#if defined(CONFIG_STM32_TIM1_FULL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN7)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN9)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN11)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN13)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN14)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTE|GPIO_PIN15)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN8)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN10)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTE|GPIO_PIN12)
#elif defined(CONFIG_STM32_TIM1_PARTIAL_REMAP)
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM1_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN12)
#  define GPIO_TIM1_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN8)
#  define GPIO_TIM1_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_TIM1_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN10)
#  define GPIO_TIM1_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN11)
#  define GPIO_TIM1_BKIN    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_TIM1_CH1NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_TIM1_CH2NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#  define GPIO_TIM1_CH3NOUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)
#endif

#if defined(CONFIG_STM32_TIM2_FULL_REMAP)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_1)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN3)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#elif defined(CONFIG_STM32_TIM2_PARTIAL_REMAP_2)
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN11)
#else
#  define GPIO_TIM2_ETR     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_TIM2_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_TIM2_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM2_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM2_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#endif

#if defined(CONFIG_STM32_TIM3_FULL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#elif defined(CONFIG_STM32_TIM3_PARTIAL_REMAP)
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN4)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN5)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM3_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM3_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM3_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM3_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM3_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#endif

#if defined(CONFIG_STM32_TIM4_REMAP)
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN13)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN14)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN15)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN15)
#else
#  define GPIO_TIM4_CH1IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH1OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_TIM4_CH2IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH2OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)
#  define GPIO_TIM4_CH3IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH3OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM4_CH4IN   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_TIM4_CH4OUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#endif

#define GPIO_TIM5_CH1IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM5_CH1OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN0)
#define GPIO_TIM5_CH2IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM5_CH2OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#define GPIO_TIM5_CH3IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM5_CH3OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#define GPIO_TIM5_CH4IN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#define GPIO_TIM5_CH4OUT    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)

#if defined(CONFIG_STM32_TIM12_REMAP)
#  define GPIO_TIM12_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_TIM12_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_TIM12_CH2IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_TIM12_CH2OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN13)
#else
#  define GPIO_TIM12_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN4)
#  define GPIO_TIM12_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN4)
#  define GPIO_TIM12_CH2IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN5)
#  define GPIO_TIM12_CH2OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN5)
#endif

#if defined(CONFIG_STM32_TIM13_REMAP)
#  define GPIO_TIM13_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0)
#  define GPIO_TIM13_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0)
#else
#  define GPIO_TIM13_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN8)
#  define GPIO_TIM13_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN8)
#endif

#if defined(CONFIG_STM32_TIM14_REMAP)
#  define GPIO_TIM14_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1)
#  define GPIO_TIM14_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN1)
#else
#  define GPIO_TIM14_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9)
#  define GPIO_TIM14_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN9)
#endif

#if defined(CONFIG_STM32_TIM15_REMAP)
#  define GPIO_TIM15_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN14)
#  define GPIO_TIM15_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#  define GPIO_TIM15_CH2IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN15)
#  define GPIO_TIM15_CH2OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)
#else
#  define GPIO_TIM15_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM15_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_TIM15_CH2IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_TIM15_CH2OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN3)
#endif
#define GPIO_TIM15_BKIN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN9)
#define GPIO_TIM15_CH1NOUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN15)

#if defined(CONFIG_STM32_TIM16_REMAP)
#  define GPIO_TIM16_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN6)
#  define GPIO_TIM16_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN6)
#else
#  define GPIO_TIM16_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_TIM16_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN8)
#endif
#define GPIO_TIM16_BKIN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN5)
#define GPIO_TIM16_CH1NOUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)

#if defined(CONFIG_STM32_TIM17_REMAP)
#  define GPIO_TIM17_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN7)
#  define GPIO_TIM17_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN7)
#else
#  define GPIO_TIM17_CH1IN  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_TIM17_CH1OUT (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN9)
#endif
#define GPIO_TIM17_BKIN     (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#define GPIO_TIM17_CH1NOUT  (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN7)

/* USART */

#if defined(CONFIG_STM32_USART1_REMAP)
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN6)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
#else
#  define GPIO_USART1_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN9)
#  define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN10)
#endif

#if defined(CONFIG_STM32_USART2_REMAP)
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN3)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN4)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN5)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN6)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN7)
#else
#  define GPIO_USART2_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN0)
#  define GPIO_USART2_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN1)
#  define GPIO_USART2_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN2)
#  define GPIO_USART2_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN3)
#  define GPIO_USART2_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN4)
#endif

#if defined(CONFIG_STM32_USART3_FULL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN8)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN9)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN10)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTD|GPIO_PIN12)
#elif defined(CONFIG_STM32_USART3_PARTIAL_REMAP)
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#else
#  define GPIO_USART3_TX    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN10)
#  define GPIO_USART3_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#  define GPIO_USART3_CK    (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12)
#  define GPIO_USART3_CTS   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN13)
#  define GPIO_USART3_RTS   (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN14)
#endif

#define GPIO_UART4_TX       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN10)
#define GPIO_UART4_RX       (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN11)

#define GPIO_UART5_TX       (GPIO_ALT|GPIO_CNF_AFPP|GPIO_MODE_50MHz|GPIO_PORTC|GPIO_PIN12)
#define GPIO_UART5_RX       (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN2)

#endif /* __ARCH_ARM_SRC_STM32_HARDWARE_STM32F100_PINMAP_H */
