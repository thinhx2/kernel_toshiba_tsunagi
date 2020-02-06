/* arch/arm/mach-msm/board-htcleo.h
 *
 * Copyright (C) 2009 HTC Corporation.
 * Author: Haley Teng <Haley_Teng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
*/

#ifndef __ARCH_ARM_MACH_MSM_BOARD_TSUNAGI_H
#define __ARCH_ARM_MACH_MSM_BOARD_TSUNAGI_H

#include <mach/board.h>
// As first set to 128 MB, ram cosole behind
#define MSM_EBI1_BANK0_BASE     0x11800000
#define MSM_EBI1_BANK0_SIZE     0x0E800000

/* Don't change that */
#define MSM_SMI_BASE		0x00000000
#define MSM_SMI_SIZE		0x04000000

/* Begin SMI region */
/* First part of SMI is used for OEMSBL & AMSS */
#define MSM_PMEM_SMI_BASE	(MSM_SMI_BASE + 0x02B00000)
#define MSM_PMEM_SMI_SIZE	0x01500000

#define MSM_FB_BASE		MSM_PMEM_SMI_BASE
#define MSM_FB_SIZE		0x00300000

#define MSM_GPU_PHYS_BASE	(MSM_PMEM_SMI_BASE + MSM_FB_SIZE)
#define MSM_GPU_PHYS_SIZE	0x00300000

#define MSM_PMEM_VENC_BASE    (MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE)
#define MSM_PMEM_VENC_SIZE	0x006c0000

#define MSM_PMEM_ADSP_BASE    (MSM_GPU_PHYS_BASE + MSM_GPU_PHYS_SIZE+MSM_PMEM_VENC_SIZE)
#define MSM_PMEM_ADSP_SIZE	0x00800000


// MSM_RAM_CONSOLE uses the last 0x00040000 of EBI memory, defined in msm_iomap.h
#define MSM_RAM_CONSOLE_SIZE    0x00040000
//#define MSM_RAM_CONSOLE_BASE    (MSM_PMEM_SMI_BASE + MSM_FB_SIZE + MSM_GPU_PHYS_SIZE + MSM_PMEM_VENC_SIZE ) //0x03C00000
/* End SMI region */

/* Begin EBI region */
#define PMEM_KERNEL_EBI1_SIZE	0x00028000

#define MSM_PMEM_SF_SIZE        0x02000000

#define MSM_PANEL_PHYS        0x00080000
#define MSM_PANEL_SIZE        0x00080000

/* End EBI region */


#define TSUNAGI_GPIO_KP_VOLUP		41
#define TSUNAGI_GPIO_KP_VOLDOWN		40
#define TSUNAGI_GPIO_KP_CAMERA		39

/* Bluetooth */
#define TSUNAGI_BT_PCM_OUT     		68
#define TSUNAGI_BT_PCM_IN      		69
#define TSUNAGI_BT_PCM_SYNC    		70
#define TSUNAGI_BT_PCM_CLK    		71

#define TSUNAGI_GPIO_BT_UART1_RTS    	43
#define TSUNAGI_GPIO_BT_UART1_CTS    	44
#define TSUNAGI_GPIO_BT_UART1_RX     	45
#define TSUNAGI_GPIO_BT_UART1_TX     	46


#endif /* __ARCH_ARM_MACH_MSM_BOARD_TSUNAGI_H */
