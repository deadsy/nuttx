/****************************************************************************
 * config/imxrt1020-evk/src/imxrt_bringup.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/types.h>
#include <debug.h>

#include <syslog.h>
#include <nuttx/i2c/i2c_master.h>
#include <imxrt_lpi2c.h>
#include <imxrt_ivt.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#ifdef CONFIG_IMXRT_USDHC
#include "imxrt_usdhc.h"
#endif

#ifdef CONFIG_IMXRT_ENET
#include "imxrt_enet.h"
#endif

#ifdef CONFIG_IMXRT_LPSPI
#include "nuttx/spi/spi_transfer.h"
#include <imxrt_lpspi.h>
#endif

#include "imxrt1020-evk.h"

#include <arch/board/board.h>   /* Must always be included last */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#define MMCSD_MINOR 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C)
static void imxrt_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = imxrt_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          imxrt_i2cbus_uninitialize(i2c);
        }
    }
}
#endif /*CONFIG_I2C_DRIVER && CONFIG_IMXRT_LPI2C */

#if defined(CONFIG_SPI_DRIVER) && defined(CONFIG_IMXRT_LPSPI)
static void imxrt_spi_register(int bus)
{
  FAR struct spi_dev_s *spi;
  int ret;

  spi = imxrt_lpspibus_initialize(bus);
  if (spi == NULL)
    {
      serr("ERROR: Failed to get SPI%d interface\n", bus);
    }
  else
    {
      ret = spi_register(spi, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register SPI%d driver: %d\n", bus, ret);
        }
    }
}
#endif /*CONFIG_SPI_DRIVER && CONFIG_IMXRT_LPSPI */

#ifdef CONFIG_IMXRT_USDHC
static int nsh_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc;
  int ret = 0;

  /* Get an instance of the SDIO interface */

  sdmmc = imxrt_usdhc_initialize(0);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(0, sdmmc);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
        }
    }
  return OK;
}
#endif /*CONFIG_IMXRT_USDHC */

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

int imxrt_bringup(void)
{
  int ret;

  /* Need to reference the IVT to stop the compiler being over-zealous in
   * removing it....despite all the KEEP instructions etc!  Feel free to
   * remove this line if you don't believe me, but don't just look at the
   * map file for confirmation, see what's at offset 0x1000 in the image
   * (clue: nothing, nada, zilch...it's not there).
   */
  ret = (g_boot_data.start == 0);

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_DEV_GPIO
  ret = imxrt_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif /*CONFIG_DEV_GPIO */

#ifdef CONFIG_I2C_DRIVER
#ifdef CONFIG_IMXRT_LPI2C1
  imxrt_i2c_register(1);
#endif
#ifdef CONFIG_IMXRT_LPI2C2
  imxrt_i2c_register(2);
#endif
#ifdef CONFIG_IMXRT_LPI2C3
  imxrt_i2c_register(3);
#endif
#ifdef CONFIG_IMXRT_LPI2C4
  imxrt_i2c_register(4);
#endif
#endif /* CONFIG_I2C_DRIVER */

#ifdef CONFIG_SPI_DRIVER
#ifdef CONFIG_IMXRT_LPSPI1
  imxrt_config_gpio(GPIO_LPSPI1_CS);
  imxrt_spi_register(1);
#endif
#ifdef CONFIG_IMXRT_LPSPI2
  imxrt_config_gpio(GPIO_LPSPI2_CS);
  imxrt_spi_register(2);
#endif
#ifdef CONFIG_IMXRT_LPSPI3
  imxrt_config_gpio(GPIO_LPSPI3_CS);
  imxrt_spi_register(3);
#endif
#ifdef CONFIG_IMXRT_LPSPI4
  imxrt_config_gpio(GPIO_LPSPI4_CS);
  imxrt_spi_register(4);
#endif
#endif /* CONFIG_SPI_DRIVER */

#ifdef CONFIG_FS_PROCFS
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif /*CONFIG_FS_PROCFS */

  return ret;
}