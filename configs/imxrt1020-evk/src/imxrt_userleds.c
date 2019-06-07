/****************************************************************************
 * configs/imxrt1020-evk/src/imxrt_userleds.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

/*
 * TODO There are four LED status indicators.
 * The functions of these LEDs are:
 *
 *   - Main 3v3 Supply (LED4, D11)
 *   - LED1 D1  GPIO_EMC_21 (GPIO2 bit 21)
 *   - LED2 D3  GPIO_EMC_22 (GPIO2 bit 22)
 *   - LED3 D5  GPIO_EMC_24 (GPIO2 bit 24)
 *
 * LED1-3 are under software control
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"
#include "imxrt1020-evk.h"

#include <arch/board/board.h>

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

void board_userled_initialize(void)
{
  /* Configure LED GPIO for output */

  imxrt_config_gpio(GPIO_LED1);
  imxrt_config_gpio(GPIO_LED2);
  imxrt_config_gpio(GPIO_LED3);
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch(led)
    {
    case 0:
      imxrt_gpio_write(GPIO_LED1, !ledon);  /* Low illuminates */
      break;
    case 1:
      imxrt_gpio_write(GPIO_LED2, !ledon);  /* Low illuminates */
      break;
    case 2:
      imxrt_gpio_write(GPIO_LED3, !ledon);  /* Low illuminates */
      break;
    default:
      break;
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
  /* Low illuminates */
  
  imxrt_gpio_write(GPIO_LED1, (ledset & BOARD_USERLED1_BIT));
  imxrt_gpio_write(GPIO_LED2, (ledset & BOARD_USERLED2_BIT));
  imxrt_gpio_write(GPIO_LED3, (ledset & BOARD_USERLED3_BIT));
}

#endif /* !CONFIG_ARCH_LEDS */
