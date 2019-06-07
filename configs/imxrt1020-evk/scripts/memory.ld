/****************************************************************************
 * configs/imxrt1020-evk/scripts/memory.ld
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Jason T. Harris <sirmanlypowers@gmail.com>
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
 * On Chip RAM:
 *
 * The i.MX RT1020 has 256 KiB of on chip RAM.
 * The FlexRAM controller partitions it into 3 regions:
 *
 * ocram - on chip ram (@ 0x2020:0000)
 * dtcm - data tightly coupled memory (@ 0x2000:0000)
 * itcm - instruction tightly coupled memory (@ 0x0000:0000)
 * 
 * The RAM split is configurable:
 * At boot time it's determined by the an entry in the fusemap.
 * OCOTP.HW_OCOTP_MISC_CONF0 (0x401f:46d0)
 * It can be reconfigured by software:
 * IOMUXC_GPR.GPR16 (0x400a:c040)
 * IOMUXC_GPR.GPR17 (0x400a:c044)
 * 
 * The MIMXRT1020-EVK (at boot) has the on chip RAM partitioned as follows:
 * ocram - 128 KiB
 * dtcm  - 64 KiB
 * itcm  - 64 KiB
 * ...and that's what we set here, but keep aware that you can reconfigure it.
 * 
 * Also keep in mind that the idea is to use the external SDRAM as the main RAM
 * resource. That's relatively slow, but the L1 cache should keep things fast.
 * The on-chip RAM is go-fast RAM for optimisation. Benchmark your code and work
 * out where you want to put code/data. 
 *
 * External RAM:
 *
 * The MIMRXRT1020-EVK has a 32 MiB SDRAM (@ 0x8000:0000)
 * It is enabled and configured via the SEMC controller.
 *
 * On Chip ROM:
 *
 * The i.MX RT1020 has 96 KiB of on chip boot rom (@ 0x0020:0000)
 *
 * External ROM:
 *
 * The MIMRXRT1020-EVK has a 8 MiB of QSPI Flash (@ 0x6000:0000)
 * It is enabled and configured via the FlexSPI controller.
 */

/* Specify the memory areas */

MEMORY
{
  /* 8 MiB of QSPI Flash */
  kflash (rx)  : ORIGIN = 0x60000000, LENGTH = 1M
  uflash (rx)  : ORIGIN = 0x60100000, LENGTH = 1M
  flash  (rx)  : ORIGIN = 0x60200000, LENGTH = 6M

  /* 256 KiB of on chip RAM */
  itcm   (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  dtcm   (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
  kocram (rwx) : ORIGIN = 0x20200000, LENGTH = 64K
  uocram (rwx) : ORIGIN = 0x20210000, LENGTH = 64K

  /* 32 MiB of SDRAM */
  sdram  (rwx) : ORIGIN = 0x80000000, LENGTH = 32M
}
