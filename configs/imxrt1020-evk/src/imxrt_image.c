
/****************************************************************************
 * configs/imxrt1020-evk/src/imxrt_image.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Ivan Ucherdzhiev <ivanucherdjiev@gmail.com>
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

#include <imxrt_fcb.h>
#include <imxrt_ivt.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

__attribute__ ((section(".boot_hdr.conf")))
const struct flexspi_nor_config_s flash_config =
{
  .mem_config =
  {
    .tag = FLEXSPI_CFG_BLK_TAG,
    .version = FLEXSPI_CFG_BLK_VERSION,
    .read_sample_clksrc = FLASH_READ_SAMPLE_CLK_LOOPBACK_INTERNELLY,
    .cs_hold_time = 3u,
    .cs_setup_time = 3u,
    .device_mode_cfg_enable = true,
    .device_mode_seq.seq_num = 1,
    .device_mode_seq.seq_id = 4,        /* These commands set the Quad bit */
    .device_mode_arg = 0x40,            /* on the flash to drive 4 pins.  */
    .device_type = FLEXSPI_DEVICE_TYPE_SERIAL_NOR,
    .sflash_pad_type = SERIAL_FLASH_4PADS,
    .serial_clk_freq = FLEXSPI_SERIAL_CLKFREQ_100MHz,
    .sflash_a1size = 8u * 1024u * 1024u,
    .data_valid_time = { 16u, 16u   },
    /*
     * .controller_misc_option = (1 <<
     * FLEXSPIMISC_OFFSET_PAD_SETTING_OVERRIDE_EN),
     * .cspad_setting_override = 1|(1<<3)|(3<<6),
     * .sclkpad_setting_override = (7<<3)|(3<<6),
     * .datapad_setting_override = (2<<3)|(3<<6),
     * .dqspad_setting_override = 0,
     */
    .lookup_table =
    {
      /* 0 - Quad Input/output read sequence - with optimised XIP support */
      [0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18),
      [1] = FLEXSPI_LUT_SEQ(MODE8_SDR, FLEXSPI_4PAD, 0xA0, DUMMY_SDR, FLEXSPI_4PAD, 0x04),
      [2] = FLEXSPI_LUT_SEQ(READ_SDR, FLEXSPI_4PAD, 0x04, JMP_ON_CS, 0, 1),
      /* 1 - Read Status */
      [1 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x01),
      /* 3 - Write Enable */
      [3 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, 0, 0),
      /* 4 - Write status */
      [4 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x01, WRITE_SDR, FLEXSPI_1PAD, 0x1),
      /* 5 - Erase Sector */
      [5 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD7, RADDR_SDR, FLEXSPI_1PAD, 0x18),
      /* 9 - Page Program */
      [9 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18),
      [9 * 4 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x8, STOP, FLEXSPI_1PAD, 0x0),
      /* 11 - Chip Erase */
      [11 * 4] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xC7, STOP, FLEXSPI_1PAD, 0x0),
    },
  },
  .page_size = 256u,
  .sector_size = 4u * 1024u,
  .blocksize = 32u * 1024u,
  .is_uniform_blocksize = false,
};

__attribute__ ((section(".boot_hdr.ivt")))
const struct ivt_s g_image_vector_table =
{
  IVT_HEADER,                       /* IVT Header */
  0x60002000,                       /* Image Entry Function */
  IVT_RSVD,                         /* Reserved = 0 */
  (uint32_t)DCD_ADDRESS,            /* Address where DCD information is stored */
  (uint32_t)BOOT_DATA_ADDRESS,      /* Address where BOOT Data Structure is stored */
  (uint32_t)&g_image_vector_table,  /* Pointer to IVT Self (absolute address) */
  (uint32_t)CSF_ADDRESS,            /* Address where CSF file is stored */
  IVT_RSVD                          /* Reserved = 0 */
};

__attribute__ ((section(".boot_hdr.boot_data")))
const struct boot_data_s g_boot_data =
{
  FLASH_BASE,               /* boot start location */
  (FLASH_END - FLASH_BASE), /* size */
  PLUGIN_FLAG,              /* Plugin flag */
  0xFFFFFFFF                /* empty - extra data word */
};
