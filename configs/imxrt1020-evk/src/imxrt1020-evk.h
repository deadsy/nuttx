/****************************************************************************
 * configs/imxrt1020-evk/src/imxrt1020-evk.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Dave Marples <dave@marples.net>
 *            Jason T. Harris <sirmanlypowers@gmail.com>
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

#ifndef __CONFIGS_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H
#define __CONFIGS_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <arch/irq.h>
#include <nuttx/irq.h>

#include "imxrt_gpio.h"
#include "imxrt_iomuxc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Configuration
 */

/* LEDS
 * There are four LED status indicators located on the EVK Board.  The
 * functions of these LEDs include:
 *
 * - Main Power Supply(D3)
 *   Green: DC 5V main supply is normal.
 *   Red:   J2 input voltage is over 5.6V.
 *   Off:   The board is not powered.
 * - Reset RED LED(D15)
 * - OpenSDA LED(D16)
 * - USER LED(D5)
 *
 * Only a single LED, D5, is under software control.  It connects to
 * GPIO_AD_B0_05 which is shared with JTAG_nTRST.
 * This pin must be configured as ALT5, GPIO1_IO05
 */

#define IOMUX_LED (IOMUX_PULL_NONE | IOMUX_CMOS_OUTPUT | \
                   IOMUX_DRIVE_40OHM | IOMUX_SPEED_MEDIUM | \
                   IOMUX_SLEW_SLOW)

#define GPIO_LED (GPIO_OUTPUT | GPIO_OUTPUT_ZERO | GPIO_PORT1 | \
                  GPIO_PIN5 | IOMUX_LED)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

#if defined(CONFIG_LIB_BOARDCTL) || defined(CONFIG_BOARD_LATE_INITIALIZE)
int imxrt_bringup(void);
#endif

/****************************************************************************
 * Name: imxrt_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ****************************************************************************/

void imxrt_spidev_initialize(void);

/*****************************************************************************
 * Name: imxrt_mmcsd_spi_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

#ifdef CONFIG_MMCSD_SPI
int imxrt_mmcsd_spi_initialize(int minor);
#endif

/****************************************************************************
 * Name: imxrt_autoled_initialize
 *
 * Description:
 *   Initialize NuttX-controlled LED logic
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void imxrt_autoled_initialize(void);
#endif

#ifdef CONFIG_DEV_GPIO

/****************************************************************************
 * Name: imxrt_gpio_initialize
 *
 * Description:
 *   Initialize GPIO drivers for use with /apps/examples/gpio
 *
 ****************************************************************************/

int imxrt_gpio_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_IMXRT1020_EVK_SRC_IMXRT1020_EVK_H */
