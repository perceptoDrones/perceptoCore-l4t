/*
 * arch/arm/mach-tegra/board-percepto1.h
 *
 * Copyright (C) 2015-2016, VisionCortex Ltd.
 * Based on board-ardbeg.c:
 * Copyright (c) 2013-2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _MACH_TEGRA_BOARD_PERCEPTO1_H
#define _MACH_TEGRA_BOARD_PERCEPTO1_H

#include <mach/gpio-tegra.h>
#include <mach/irqs.h>
#include "gpio-names.h"

int ardbeg_emc_init(void);
int ardbeg_display_init(void);
int percepto1_sdhci_init(void);
int percepto1_sensors_init(void);
int ardbeg_regulator_init(void);
int ardbeg_suspend_init(void);
int ardbeg_rail_alignment_init(void);
int ardbeg_edp_init(void);
int ardbeg_panel_init(void);
int laguna_regulator_init(void);
int norrin_soctherm_init(void);

/* generated soc_therm OC interrupts */
#define TEGRA_SOC_OC_IRQ_BASE	TEGRA_NR_IRQS
#define TEGRA_SOC_OC_NUM_IRQ	TEGRA_SOC_OC_IRQ_MAX

#define UTMI1_PORT_OWNER_XUSB   0x1
#define UTMI2_PORT_OWNER_XUSB   0x2
#define HSIC1_PORT_OWNER_XUSB   0x4
#define HSIC2_PORT_OWNER_XUSB   0x8

#endif
