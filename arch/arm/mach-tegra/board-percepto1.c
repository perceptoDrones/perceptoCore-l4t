/*
 * arch/arm/mach-tegra/board-percepto1.c
 *
 * Copyright (C) 2015-2016, VisionCortex Ltd.
 * Based on board-ardbeg.c:
 * Copyright (c) 2013-2015, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/i2c-hid.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/memblock.h>
#include <linux/spi/spi-tegra.h>
#include <linux/regulator/consumer.h>
#include <linux/leds.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/edp.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/clk/tegra.h>
#include <media/tegra_dtv.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/irqchip/tegra.h>
#include <linux/can/platform/mcp251x.h>

#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t12.h>
#include <mach/io_dpd.h>
#include <mach/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/dc.h>
#include <mach/tegra_usb_pad_ctrl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio-tegra.h>
#include <mach/xusb.h>

#include "board.h"
#include "board-percepto1.h"
#include "board-common.h"
#include "clock.h"
#include "common.h"
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "tegra-of-dev-auxdata.h"

static struct board_info board_info, display_board_info;

static __initdata struct tegra_clk_init_table ardbeg_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	48000000,	false},
	{ "pll_a",	"pll_p_out1",	282240000,	false},
	{ "pll_a_out0",	"pll_a",	12288000,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	12288000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "vi_sensor2",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "sbc1",	"pll_p",	25000000,	false},
	{ "sbc2",	"pll_p",	25000000,	false},
	{ "sbc3",	"pll_p",	25000000,	false},
	{ "sbc4",	"pll_p",	25000000,	false},
	{ "sbc5",	"pll_p",	25000000,	false},
	{ "sbc6",	"pll_p",	25000000,	false},
	{ "uarta",	"pll_p",	408000000,	false},
	{ "uartb",	"pll_p",	408000000,	false},
	{ "uartc",	"pll_p",	408000000,	false},
	{ "uartd",	"pll_p",	408000000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct i2c_board_info __initdata i2c_fpga_board_info = {
	I2C_BOARD_INFO("latticeFPGA", 0x05),
	.platform_data  = 0,
};

static void percepto1_i2c_init( void )
{
    /* Percepto i2c device */
    /* Bus numbers: GEN1 is 0, GEN2 is 1 */
    i2c_register_board_info(1, &i2c_fpga_board_info, 1 );
}

static struct tegra_serial_platform_data percepto1_uartd_pdata = {
	.dma_req_selector = 19,
	.modem_interrupt = false,
};

static void __init percepto1_uart_init(void)
{
	tegra_uartd_device.dev.platform_data = &percepto1_uartd_pdata;
	if (!is_tegra_debug_uartport_hs()) {
	    int debug_port_id = uart_console_debug_init(3);
	    if (debug_port_id < 0)
		return;

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
#if !defined(CONFIG_TRUSTED_FOUNDATIONS) && \
	defined(CONFIG_ARCH_TEGRA_12x_SOC) && defined(CONFIG_FIQ_DEBUGGER)
	    tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_AVP, NULL, -1, -1);
	    platform_device_register(uart_console_debug_device);
#else
	    tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
#endif
#else
		platform_device_register(uart_console_debug_device);
#endif
	} else {
	    tegra_uartd_device.dev.platform_data = &percepto1_uartd_pdata;
	    platform_device_register(&tegra_uartd_device);
	}
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device *percepto1_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
#if defined(CONFIG_TEGRA_WAKEUP_MONITOR)
	&tegratab_tegra_wakeup_monitor_device,
#endif
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_WATCHDOG)
	&tegra_wdt0_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE) && !defined(CONFIG_USE_OF)
	&tegra12_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_hda_device,
	&tegra_offload_device,
	&tegra30_avp_audio_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&tegra_hier_ictlr_device,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = true,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
		.xcvr_hsslew_lsb = 2,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
	.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void percepto1_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	int modem_id = tegra_get_modem_id();
	struct board_info bi;
	tegra_get_pmu_board_info(&bi);

	if (board_info.sku == 1100 )
	    tegra_ehci1_utmi_pdata.u_data.host.turn_off_vbus_on_lp0 = true;

	if (board_info.major_revision >= 'A' && board_info.major_revision <= 'D' ) {
	    tegra_udc_pdata.id_det_type = TEGRA_USB_VIRTUAL_ID;
	    tegra_ehci1_utmi_pdata.id_det_type =
		TEGRA_USB_VIRTUAL_ID;
	} else {
	    tegra_udc_pdata.id_det_type = TEGRA_USB_PMU_ID;
	    tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_PMU_ID;
	}
	tegra_ehci1_utmi_pdata.id_extcon_dev_name = "as3722-extcon";

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
	    tegra_otg_pdata.is_xhci = false;
	    tegra_udc_pdata.u_data.dev.is_xhci = false;
	} else {
	    tegra_otg_pdata.is_xhci = true;
	    tegra_udc_pdata.u_data.dev.is_xhci = true;
	}
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		if (!modem_id) {
		    tegra_ehci2_device.dev.platform_data =
			&tegra_ehci2_utmi_pdata;
		    platform_device_register(&tegra_ehci2_device);
		}
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
	    tegra_ehci3_device.dev.platform_data =
		&tegra_ehci3_utmi_pdata;
	    platform_device_register(&tegra_ehci3_device);
	}
}

static struct tegra_xusb_platform_data xusb_pdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0 | TEGRA_XUSB_SS_P1 |
			TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_USB2_P2,
};

#ifdef CONFIG_TEGRA_XUSB_PLATFORM
static void percepto1_xusb_init(void)
{
		int usb_port_owner_info = tegra_get_usb_port_owner_info();

		xusb_pdata.lane_owner = (u8) tegra_get_lane_owner_info();

		if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0);

		if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_SS_P0);

		if (usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)
				xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P0;

		if (usb_port_owner_info & HSIC2_PORT_OWNER_XUSB)
				xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P1;
}
#endif

#ifdef CONFIG_USE_OF
static struct of_dev_auxdata percepto1_auxdata_lookup[] __initdata = {
	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-se", 0x70012000, "tegra12-se", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
		"gk20a.0", NULL),
#ifdef CONFIG_ARCH_TEGRA_VIC
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006040, "serial-tegra.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006200, "serial-tegra.2",
				NULL),
	T124_I2C_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci",
				&xusb_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dfll", 0x70040084, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-camera", 0, "pcl-generic",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-ahci-sata", 0x70027000, "tegra-sata.0",
		NULL),
	{}
};
#endif

static struct mcp251x_platform_data mcp2515_pdata = {
	.oscillator_frequency	= 20*1000*1000,
	.board_specific_setup	= 0,
	.power_enable		= 0,
};

static struct tegra_spi_device_controller_data mcp2512_dev_cdata = {
	.rx_clk_tap_delay = 0,
	.is_hw_based_cs = true,
	.tx_clk_tap_delay = 0,
};

static struct spi_board_info percepto_spi_board_info[] = {

	/* SPI-to-CAN transceiver */
	[0] = {
		.modalias	= "mcp2515",
		.platform_data	= &mcp2515_pdata,
		.controller_data = &mcp2512_dev_cdata,
		.max_speed_hz	= 10*1000*1000,
		.bus_num	= 0,
		.mode		= SPI_MODE_0,
		.chip_select	= 0,
	},

	/* This is a generic userspace accesible device
	   connected to the MachXO2 programming interface on Rev.D boards */
	[1] = {
		.modalias = "spidev",
		.platform_data = NULL,
		.max_speed_hz = 1*1000*1000,
		.bus_num = 3,
		.mode = SPI_MODE_0,
		.chip_select = 0,
	}
};

static int __init percepto1_spi_devices_init(void)
{
	 int ret;

	 /* Interrupt pin available from board revision Percepto 1.4 */
	 /* Earlier revisions have it N/C */
	 percepto_spi_board_info[0].irq = gpio_to_irq(TEGRA_GPIO_PX3);

   ret = gpio_request(TEGRA_GPIO_PX3, "MCP251x interrupt");

	 if (ret < 0) {
		 pr_err("gpio_request failed for MCP251x irq\n");
		 return ret;
	 }

	 gpio_direction_input(TEGRA_GPIO_PX3);
	 spi_register_board_info( percepto_spi_board_info, ARRAY_SIZE(percepto_spi_board_info) );
	 return 0;
}

static void __init edp_init(void)
{
    ardbeg_edp_init();
}

static void __init tegra_percepto1_early_init(void)
{
	tegra_clk_init_from_table(ardbeg_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("percepto1");
}

static struct tegra_dtv_platform_data percepto1_dtv_pdata = {
	.dma_req_selector = 11,
};

static void __init percepto1_dtv_init(void)
{
	tegra_dtv_device.dev.platform_data = &percepto1_dtv_pdata;
	platform_device_register(&tegra_dtv_device);
}

static struct tegra_io_dpd pexbias_io = {
	.name			= "PEX_BIAS",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 4,
};
static struct tegra_io_dpd pexclk1_io = {
	.name			= "PEX_CLK1",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 5,
};
static struct tegra_io_dpd pexclk2_io = {
	.name			= "PEX_CLK2",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 6,
};

extern unsigned long long tegra_chip_uid(void);
static unsigned long long tegra_uid;
static struct kobject *uid_kobj;

// SysFs callback: Read chip uid in decimal
static ssize_t uid_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
  return sprintf(buf, "%llu\n", tegra_uid );
}

// SysFs callback: Read chip uid in hex
static ssize_t uid_show_hex(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
  return sprintf(buf, "0x%llx\n", tegra_uid );
}

// Setup of permissions and callbacks for sysfs.
static struct kobj_attribute uid_attribute =__ATTR( soc_uid, 0666, uid_show, NULL );
static struct kobj_attribute uid_attribute_hex =__ATTR( soc_uid_hex, 0666, uid_show_hex, NULL );

static int tegra_percepto1_uid_init(void)
{
	tegra_uid = tegra_chip_uid();

	pr_info( "chip UID: %llu\n", tegra_uid );

	// Create sysfs entry
	uid_kobj = kobject_create_and_add("percepto", kernel_kobj );

	if ( !uid_kobj )
		return -ENOMEM;

	// Create sysfs files
	if ( sysfs_create_file( uid_kobj, &uid_attribute.attr ) )
	{
		pr_debug( "failed to create sysfs entry for chip uid\n" );
		kobject_put(uid_kobj);
		return -ENOMEM;
	}

	if ( sysfs_create_file( uid_kobj, &uid_attribute_hex.attr ) )
	{
		pr_debug( "failed to create sysfs entry for chip uid (hex)\n" );
		kobject_put(uid_kobj);
		return -ENOMEM;
	}

	return 0;
}

static void __init tegra_percepto1_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

	ardbeg_display_init();
	percepto1_uart_init();
	percepto1_usb_init();
	percepto1_xusb_init();
	percepto1_i2c_init();
	platform_add_devices(percepto1_devices, ARRAY_SIZE(percepto1_devices));
	tegra_io_dpd_init();
	percepto1_sdhci_init();
	laguna_regulator_init();
	percepto1_dtv_init();
	ardbeg_suspend_init();
	ardbeg_emc_init();
	edp_init();
	isomgr_init();
	ardbeg_panel_init();

	/* put PEX pads into DPD mode to save additional power */
	tegra_io_dpd_enable(&pexbias_io);
	tegra_io_dpd_enable(&pexclk1_io);
	tegra_io_dpd_enable(&pexclk2_io);;

#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	percepto1_sensors_init();
	norrin_soctherm_init();
	percepto1_spi_devices_init();
	tegra_percepto1_uid_init();
}

static void __init tegra_percepto1_init_early(void)
{
	tegra_get_board_info(&board_info);
	ardbeg_rail_alignment_init();
	tegra12x_init_early();
}

static void __init tegra_percepto1_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_percepto1_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, percepto1_auxdata_lookup,
		&platform_bus);
#endif
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

	tegra_percepto1_late_init();
}

static void __init tegra_percepto1_reserve(void)
{
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	ulong tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) || defined(CONFIG_TEGRA_NO_CARVEOUT)
	ulong carveout_size = 0;
	ulong fb2_size = SZ_16M;
#else
	ulong carveout_size = SZ_1G;
	ulong fb2_size = SZ_4M;
#endif
	ulong fb1_size = SZ_16M + SZ_2M;
	ulong vpr_size = 186 * SZ_1M;

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* support FBcon on 4K monitors */
	fb2_size = SZ_64M + SZ_8M;	/* 4096*2160*4*2 = 70778880 bytes */
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	tmp = fb1_size;
	fb1_size = fb2_size;
	fb2_size = tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

	tegra_reserve4(carveout_size, fb1_size, fb2_size, vpr_size);
}

static const char * const percepto1_board_compat[] = {
	"nvidia,percepto1",
	NULL
};

DT_MACHINE_START(PERCEPTO1, "percepto1")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_percepto1_reserve,
	.init_early	= tegra_percepto1_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_percepto1_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= percepto1_board_compat,
	.init_late      = tegra_init_late
MACHINE_END
