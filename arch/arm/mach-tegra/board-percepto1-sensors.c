/*
 * arch/arm/mach-tegra/board-percepto1-sensors.c
 *
 * Copyright (C) 2015-2016, VisionCortex Ltd.
 * Based on board-ardbeg-sensors.c:
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/nct1008.h>
#include <linux/pid_thermal_gov.h>
#include <linux/tegra-fuse.h>
#include <mach/edp.h>
#include <mach/pinmux-t12.h>
#include <mach/pinmux.h>

#include <linux/platform_device.h>
#include <linux/generic_adc_thermal.h>

#include "cpu-tegra.h"
#include "board.h"
#include "board-common.h"
#include "board-percepto1.h"
#include "tegra-board-id.h"

static struct mpu_platform_data mpu9250_gyro_data = {
        .int_config  = 0x00,
        .level_shifter = 0,
        .orientation = {   1,  0,  0,
                           0,  1,  0,
                           0,  0, 1 },
        .sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
        .sec_slave_id   = COMPASS_ID_AK8963,
        .secondary_i2c_addr = 0x0C,
        .secondary_orientation = { 0,  1, 0,
                                   -1, 0,  0,
                                   0,  0,  1 },
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation    = MTMAT_BOT_CCW_270,
	.config         = NVI_CONFIG_BOOT_MPU,
};

static struct i2c_board_info __initdata inv_mpu9250_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO( "mpu9250", 0x68 ),
		.platform_data = &mpu9250_gyro_data,
	},
	{
		I2C_BOARD_INFO("ak8975", 0x0c),
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = TEGRA_GPIO_PU4;
	unsigned gyro_bus_num = 0;
	char *gyro_name = "mpu9250";
	struct board_info board_info;

	pr_info("*** MPU START *** mpuirq_init...\n");

	tegra_get_board_info(&board_info);

	ret = gpio_request(TEGRA_GPIO_PU4, gyro_name);
	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu9250_i2c0_board_info[0].irq = gpio_to_irq(TEGRA_GPIO_PU4);
	i2c_register_board_info(gyro_bus_num, inv_mpu9250_i2c0_board_info,
		ARRAY_SIZE(inv_mpu9250_i2c0_board_info));
}

static struct pid_thermal_gov_params cpu_pid_params = {
	.max_err_temp = 4000,
	.max_err_gain = 1000,

	.gain_p = 1000,
	.gain_d = 0,

	.up_compensation = 15,
	.down_compensation = 15,
};

static struct thermal_zone_params cpu_tzp = {
	.governor_name = "pid_thermal_gov",
	.governor_params = &cpu_pid_params,
};

static struct thermal_zone_params board_tzp = {
	.governor_name = "pid_thermal_gov"
};

static struct nct1008_platform_data percepto1_nct72_pdata = {
	.loc_name = "tegra",
	.supported_hwrev = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.offset = 0,
	.extended_range = true,

	.sensors = {
		[LOC] = {
			.tzp = &board_tzp,
			.shutdown_limit = 120, /* C */
			.passive_delay = 1000,
			.num_trips = 1,
			.trips = {
				{
					.cdev_type = "therm_est_activ",
					.trip_temp = 40000,
					.trip_type = THERMAL_TRIP_ACTIVE,
					.hysteresis = 1000,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 1,
				},
			},
		},
		[EXT] = {
			.tzp = &cpu_tzp,
			.shutdown_limit = 95, /* C */
			.passive_delay = 1000,
			.num_trips = 2,
			.trips = {
				{
					.cdev_type = "shutdown_warning",
					.trip_temp = 93000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.mask = 0,
				},
				{
					.cdev_type = "cpu-balanced",
					.trip_temp = 83000,
					.trip_type = THERMAL_TRIP_PASSIVE,
					.upper = THERMAL_NO_LIMIT,
					.lower = THERMAL_NO_LIMIT,
					.hysteresis = 1000,
					.mask = 1,
				},
			}
		}
	}
};

static struct i2c_board_info percepto1_i2c_nct72_board_info[] = {
	{
		I2C_BOARD_INFO("nct72", 0x4c),
		.platform_data = &percepto1_nct72_pdata,
		.irq = -1,
	},
#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		I2C_BOARD_INFO("nct72", 0x4d),
		.platform_data = &percepto1_nct72_tskin_pdata,
		.irq = -1,
	}
#endif
};

static int percepto1_nct72_init(void)
{
	int nct72_port = TEGRA_GPIO_PI6;
	int ret = 0;
	int i;
	struct thermal_trip_info *trip_state;
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	/* raise NCT's thresholds if soctherm CP,FT fuses are ok */
	if ((tegra_fuse_calib_base_get_cp(NULL, NULL) >= 0) &&
	    (tegra_fuse_calib_base_get_ft(NULL, NULL) >= 0)) {
		percepto1_nct72_pdata.sensors[EXT].shutdown_limit += 20;
		for (i = 0; i < percepto1_nct72_pdata.sensors[EXT].num_trips;
			 i++) {
			trip_state = &percepto1_nct72_pdata.sensors[EXT].trips[i];
			if (!strncmp(trip_state->cdev_type, "cpu-balanced",
					THERMAL_NAME_LENGTH)) {
				trip_state->cdev_type = "_none_";
				break;
			}
		}
	} else {
		tegra_platform_edp_init(
			percepto1_nct72_pdata.sensors[EXT].trips,
			&percepto1_nct72_pdata.sensors[EXT].num_trips,
					12000); /* edp temperature margin */
		tegra_add_cpu_vmax_trips(
			percepto1_nct72_pdata.sensors[EXT].trips,
			&percepto1_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_tgpu_trips(
			percepto1_nct72_pdata.sensors[EXT].trips,
			&percepto1_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_vc_trips(
			percepto1_nct72_pdata.sensors[EXT].trips,
			&percepto1_nct72_pdata.sensors[EXT].num_trips);
		tegra_add_core_vmax_trips(
			percepto1_nct72_pdata.sensors[EXT].trips,
			&percepto1_nct72_pdata.sensors[EXT].num_trips);
	}

	percepto1_i2c_nct72_board_info[0].irq = gpio_to_irq(nct72_port);

	ret = gpio_request(nct72_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct72_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct72_port)", __func__);
		gpio_free(nct72_port);
	}

		percepto1_nct72_pdata.sensors[EXT].shutdown_limit = 105;
		percepto1_nct72_pdata.sensors[LOC].shutdown_limit = 100;
		i2c_register_board_info(0, percepto1_i2c_nct72_board_info, 1); /* only register device[0] */

		return ret;
}

struct ntc_thermistor_adc_table {
	int temp; /* degree C */
	int adc;
};

int __init percepto1_sensors_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);

	mpuirq_init();
	percepto1_nct72_init();

	return 0;
}
