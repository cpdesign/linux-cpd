/*
 * Copyright 2009 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright (C) 2009 Marc Kleine-Budde, Pengutronix
 * Copyright 2010 Creative Product Design
 *
 * Derived from mx35 3stack.
 * Original author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/memory.h>
#include <linux/gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/iomux-mx35.h>
#include <mach/irqs.h>
#include <mach/ipu.h>
#include <mach/mx3fb.h>
#include <mach/audmux.h>

#include <linux/i2c.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/mc13xxx.h>
#include <linux/gpio_keys.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/kernel.h>

#include "devices-imx35.h"
#include "devices.h"
#include "cpu_op-mx35.h"

#define GPIO_LCDPWR	IMX_GPIO_NR(1, 2)
#define GPIO_PMIC_INT	IMX_GPIO_NR(2, 0)

#define GPIO_BUTTON1	IMX_GPIO_NR(1, 4)
#define GPIO_BUTTON2	IMX_GPIO_NR(1, 5)
#define GPIO_BUTTON3	IMX_GPIO_NR(1, 7)
#define GPIO_BUTTON4	IMX_GPIO_NR(1, 8)
#define GPIO_BUTTON5	IMX_GPIO_NR(1, 9)
#define GPIO_BUTTON6	IMX_GPIO_NR(1, 10)
#define GPIO_BUTTON7	IMX_GPIO_NR(1, 11)
#define GPIO_BUTTON8	IMX_GPIO_NR(1, 12)

#define GPIO_SPEAKER	IMX_GPIO_NR(1, 14)

#define GPIO_BP_RESET	IMX_GPIO_NR(2, 18)

#define GPIO_LEDR	IMX_GPIO_NR(3, 14)
#define GPIO_LEDG	IMX_GPIO_NR(3, 15)
#define GPIO_LEDB	IMX_GPIO_NR(3, 30)

#define GPIO_SD_CD	IMX_GPIO_NR(3, 1)
#define GPIO_SD_WP	IMX_GPIO_NR(3, 2)

static long vpr200_blink(int state)
{
	gpio_direction_output(GPIO_LEDR, !state);

	return 0;
}


static const struct fb_videomode fb_modedb[] = {
	{
		/* 800x480 @ 60 Hz */
		.name		= "PT0708048",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= KHZ2PICOS(33260),
		.left_margin	= 50,
		.right_margin	= 156,
		.upper_margin	= 10,
		.lower_margin	= 10,
		.hsync_len	= 1,	/* note: DE only display */
		.vsync_len	= 1,	/* note: DE only display */
		.sync		= FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	}, {
		/* 800x480 @ 60 Hz */
		.name		= "CTP-CLAA070LC0ACW",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= KHZ2PICOS(27000),
		.left_margin	= 50,
		.right_margin	= 50,	/* whole line should have 900 clocks */
		.upper_margin	= 10,
		.lower_margin	= 10,	/* whole frame should have 500 lines */
		.hsync_len	= 1,	/* note: DE only display */
		.vsync_len	= 1,	/* note: DE only display */
		.sync		= FB_SYNC_CLK_IDLE_EN | FB_SYNC_OE_ACT_HIGH,
		.vmode		= FB_VMODE_NONINTERLACED,
		.flag		= 0,
	}
};

static struct ipu_platform_data mx3_ipu_data = {
	.irq_base = MXC_IPU_IRQ_START,
};

static struct mx3fb_platform_data mx3fb_pdata = {
	.dma_dev	= &mx3_ipu.dev,
	.name		= "PT0708048",
	.mode		= fb_modedb,
	.num_modes	= ARRAY_SIZE(fb_modedb),
};

static struct physmap_flash_data vpr200_flash_data = {
	.width  = 2,
};

static struct resource vpr200_flash_resource = {
	.start	= MX35_CS0_BASE_ADDR,
	.end	= MX35_CS0_BASE_ADDR + SZ_64M - 1,
	.flags	= IORESOURCE_MEM,
};

static struct platform_device vpr200_flash = {
	.name	= "physmap-flash",
	.id	= 0,
	.dev	= {
		.platform_data  = &vpr200_flash_data,
	},
	.resource = &vpr200_flash_resource,
	.num_resources = 1,
};

static const struct mxc_nand_platform_data
		vpr200_nand_board_info __initconst = {
	.width = 1,
	.hw_ecc = 1,
	.flash_bbt = 1,
};

static struct esdhc_platform_data sd1_pdata = {
	.wp_gpio = GPIO_SD_WP,
	.cd_gpio = GPIO_SD_CD,
};

static struct gpio_led vpr200_gpio_leds[] = {
	[0] = {
		.name			= "gpio-led:red:",
		.gpio			= GPIO_LEDR,
		.default_trigger	= "default-off",
		.active_low		= 1,
	},
	[1] = {
		.name			= "gpio-led:green:",
		.gpio			= GPIO_LEDG,
		.default_trigger	= "heartbeat",
		.active_low		= 1,
	},
	[2] = {
		.name			= "gpio-led:blue:",
		.gpio			= GPIO_LEDB,
		.default_trigger	= "default-off",
		.active_low		= 1,
	},
};

static struct gpio_led_platform_data vpr200_led_pdata = {
	.leds           = vpr200_gpio_leds,
	.num_leds       = ARRAY_SIZE(vpr200_gpio_leds),
};

static struct platform_device vpr200_led_device = {
	 .name   = "leds-gpio",
	 .id     = -1,
	 .dev    = {
		 .platform_data  =  &vpr200_led_pdata,
	},
};

#define VPR_KEY_DEBOUNCE	100
static struct gpio_keys_button vpr200_gpio_keys_table[] = {
	{KEY_F2, GPIO_BUTTON1, 1, "vpr-keys: F2", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F3, GPIO_BUTTON2, 1, "vpr-keys: F3", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F4, GPIO_BUTTON3, 1, "vpr-keys: F4", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F5, GPIO_BUTTON4, 1, "vpr-keys: F5", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F6, GPIO_BUTTON5, 1, "vpr-keys: F6", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F7, GPIO_BUTTON6, 1, "vpr-keys: F7", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F8, GPIO_BUTTON7, 1, "vpr-keys: F8", EV_KEY, 0, VPR_KEY_DEBOUNCE},
	{KEY_F9, GPIO_BUTTON8, 1, "vpr-keys: F9", EV_KEY, 0, VPR_KEY_DEBOUNCE},
};

static struct gpio_keys_platform_data vpr200_gpio_keys_data = {
	.buttons = vpr200_gpio_keys_table,
	.nbuttons = ARRAY_SIZE(vpr200_gpio_keys_table),
};

static struct platform_device vpr200_device_gpiokeys = {
	.name = "gpio-keys",
	.dev = {
		.platform_data = &vpr200_gpio_keys_data,
	}
};

static struct mc13xxx_platform_data vpr200_pmic = {
	.flags = MC13XXX_USE_ADC |
		 MC13XXX_USE_BATTERY |
		 MC13XXX_USE_TOUCHSCREEN,
};

static const struct imxi2c_platform_data vpr200_i2c0_data __initconst = {
	.bitrate = 100000,
};

static struct at24_platform_data vpr200_eeprom = {
	.byte_len = 2048 / 8,
	.page_size = 1,
};

static struct i2c_board_info vpr200_i2c_devices[] = {
	{
		I2C_BOARD_INFO("at24", 0x50), /* E0=0, E1=0, E2=0 */
		.platform_data = &vpr200_eeprom,
	}, {
		I2C_BOARD_INFO("mc13892", 0x08),
		.platform_data = &vpr200_pmic,
		.irq = gpio_to_irq(GPIO_PMIC_INT),
	}
};

static struct i2c_board_info vpr200_bus1_devices[] = {
	{
		I2C_BOARD_INFO("bmp085", 0x77),
	}, {
		I2C_BOARD_INFO("pcm1774", 0x47),
	}, {
		I2C_BOARD_INFO("isl22316", 0x2b), // 010 1011
	}
};

static iomux_v3_cfg_t vpr200_pads[] = {
	/* UART1 */
	MX35_PAD_TXD1__UART1_TXD_MUX,
	MX35_PAD_RXD1__UART1_RXD_MUX,
	/* UART3 */
	MX35_PAD_ATA_DATA10__UART3_RXD_MUX,
	MX35_PAD_ATA_DATA11__UART3_TXD_MUX,
	/* FEC */
	MX35_PAD_FEC_TX_CLK__FEC_TX_CLK,
	MX35_PAD_FEC_RX_CLK__FEC_RX_CLK,
	MX35_PAD_FEC_RX_DV__FEC_RX_DV,
	MX35_PAD_FEC_COL__FEC_COL,
	MX35_PAD_FEC_RDATA0__FEC_RDATA_0,
	MX35_PAD_FEC_TDATA0__FEC_TDATA_0,
	MX35_PAD_FEC_TX_EN__FEC_TX_EN,
	MX35_PAD_FEC_MDC__FEC_MDC,
	MX35_PAD_FEC_MDIO__FEC_MDIO,
	MX35_PAD_FEC_TX_ERR__FEC_TX_ERR,
	MX35_PAD_FEC_RX_ERR__FEC_RX_ERR,
	MX35_PAD_FEC_CRS__FEC_CRS,
	MX35_PAD_FEC_RDATA1__FEC_RDATA_1,
	MX35_PAD_FEC_TDATA1__FEC_TDATA_1,
	MX35_PAD_FEC_RDATA2__FEC_RDATA_2,
	MX35_PAD_FEC_TDATA2__FEC_TDATA_2,
	MX35_PAD_FEC_RDATA3__FEC_RDATA_3,
	MX35_PAD_FEC_TDATA3__FEC_TDATA_3,
	/* Display */
	MX35_PAD_LD0__IPU_DISPB_DAT_0,
	MX35_PAD_LD1__IPU_DISPB_DAT_1,
	MX35_PAD_LD2__IPU_DISPB_DAT_2,
	MX35_PAD_LD3__IPU_DISPB_DAT_3,
	MX35_PAD_LD4__IPU_DISPB_DAT_4,
	MX35_PAD_LD5__IPU_DISPB_DAT_5,
	MX35_PAD_LD6__IPU_DISPB_DAT_6,
	MX35_PAD_LD7__IPU_DISPB_DAT_7,
	MX35_PAD_LD8__IPU_DISPB_DAT_8,
	MX35_PAD_LD9__IPU_DISPB_DAT_9,
	MX35_PAD_LD10__IPU_DISPB_DAT_10,
	MX35_PAD_LD11__IPU_DISPB_DAT_11,
	MX35_PAD_LD12__IPU_DISPB_DAT_12,
	MX35_PAD_LD13__IPU_DISPB_DAT_13,
	MX35_PAD_LD14__IPU_DISPB_DAT_14,
	MX35_PAD_LD15__IPU_DISPB_DAT_15,
	MX35_PAD_LD16__IPU_DISPB_DAT_16,
	MX35_PAD_LD17__IPU_DISPB_DAT_17,
	MX35_PAD_D3_FPSHIFT__IPU_DISPB_D3_CLK,
	MX35_PAD_D3_DRDY__IPU_DISPB_D3_DRDY,
	MX35_PAD_CONTRAST__IPU_DISPB_CONTR,
	/* LCD Enable */
	MX35_PAD_D3_VSYNC__GPIO1_2,
	/* SDCARD */
	MX35_PAD_SD1_CMD__ESDHC1_CMD,
	MX35_PAD_SD1_CLK__ESDHC1_CLK,
	MX35_PAD_SD1_DATA0__ESDHC1_DAT0,
	MX35_PAD_SD1_DATA1__ESDHC1_DAT1,
	MX35_PAD_SD1_DATA2__ESDHC1_DAT2,
	MX35_PAD_SD1_DATA3__ESDHC1_DAT3,
	MX35_PAD_ATA_DA1__GPIO3_1,
	MX35_PAD_ATA_DA2__GPIO3_2,
	/* I2C1 */
	MX35_PAD_I2C1_CLK__I2C1_SCL,
	MX35_PAD_I2C1_DAT__I2C1_SDA,
	/* I2C2 */
	MX35_PAD_I2C2_CLK__I2C2_SCL,
	MX35_PAD_I2C2_DAT__I2C2_SDA,
	/* PMIC */
	MX35_PAD_GPIO2_0__GPIO2_0,
	/* GPIO keys */
	MX35_PAD_SCKR__GPIO1_4,
	MX35_PAD_COMPARE__GPIO1_5,
	MX35_PAD_SCKT__GPIO1_7,
	MX35_PAD_FST__GPIO1_8,
	MX35_PAD_HCKT__GPIO1_9,
	MX35_PAD_TX5_RX0__GPIO1_10,
	MX35_PAD_TX4_RX1__GPIO1_11,
	MX35_PAD_TX3_RX2__GPIO1_12,
	/* I2S */
	MX35_PAD_STXFS4__AUDMUX_AUD4_TXFS,
	MX35_PAD_STXD4__AUDMUX_AUD4_TXD,
	MX35_PAD_SRXD4__AUDMUX_AUD4_RXD,
	MX35_PAD_SCK4__AUDMUX_AUD4_TXC,
	/* speaker */
	MX35_PAD_TX1__GPIO1_14,
	/* bmp085  */
	MX35_PAD_ATA_DATA5__GPIO2_18,
	/* leds */
	MX35_PAD_USBOTG_PWR__GPIO3_14,
	MX35_PAD_USBOTG_OC__GPIO3_15,
	MX35_PAD_D3_HSYNC__GPIO3_30,
};

/* USB Device config */
static const struct fsl_usb2_platform_data otg_device_pdata __initconst = {
	.operating_mode	= FSL_USB2_DR_DEVICE,
	.phy_mode	= FSL_USB2_PHY_UTMI,
	.workaround	= FLS_USB2_WORKAROUND_ENGCM09152,
};

static int vpr200_usbh_init(struct platform_device *pdev)
{
	return mx35_initialize_usb_hw(pdev->id,
			MXC_EHCI_INTERFACE_SINGLE_UNI | MXC_EHCI_INTERNAL_PHY);
}

/* USB HOST config */
static const struct mxc_usbh_platform_data usb_host_pdata __initconst = {
	.init = vpr200_usbh_init,
	.portsc = MXC_EHCI_MODE_SERIAL,
};

static struct platform_device *devices[] __initdata = {
	&vpr200_flash,
	&vpr200_device_gpiokeys,
	&vpr200_led_device,
};

static const
struct imx_ssi_platform_data vpr200_ssi_pdata __initconst = {
	.flags = IMX_SSI_SYN | IMX_SSI_NET | IMX_SSI_USE_I2S_SLAVE,
};

static void mxc_init_pcm1774(void)
{
	struct clk *clko, *parent;
	unsigned long rate;

	clko = clk_get(NULL, "clko");
	if (IS_ERR(clko)) {
		pr_err("%s: Couldn't get clko\n", __func__);
		return;
	}
	parent = clk_get(NULL, "ckih");
	if (IS_ERR(parent)) {
		pr_err("%s: Couldn't get ckih\n", __func__);
		return;
	}
	clk_set_parent(clko, parent);
	rate = clk_round_rate(clko, 12000000);
	if (rate < 8000000 || rate > 27000000) {
		printk(KERN_ERR "Error: pcm1774 sclk freq %d out of range!\n",
		       (unsigned int)rate);
		clk_put(parent);
		clk_put(clko);
		return;
	}
	clk_set_rate(clko, rate);
	clk_enable(clko);
}

/*
 * Board specific initialization.
 */
static void __init vpr200_board_init(void)
{
	mxc_iomux_v3_setup_multiple_pads(vpr200_pads, ARRAY_SIZE(vpr200_pads));

	panic_blink = vpr200_blink;

#if defined(CONFIG_CPU_FREQ_IMX)
	get_cpu_op = mx35_get_cpu_op;
#endif

	imx35_add_fec(NULL);
	imx35_add_imx2_wdt(NULL);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	if (0 != gpio_request(GPIO_LCDPWR, "LCDPWR"))
		printk(KERN_WARNING "vpr200: Couldn't get LCDPWR gpio\n");
	else
		gpio_direction_output(GPIO_LCDPWR, 0);

	if (0 != gpio_request(GPIO_PMIC_INT, "PMIC_INT"))
		printk(KERN_WARNING "vpr200: Couldn't get PMIC_INT gpio\n");
	else
		gpio_direction_input(GPIO_PMIC_INT);

	gpio_request(GPIO_BP_RESET, "BP_RESET");
	gpio_direction_output(GPIO_BP_RESET, 1);

	if (0 != gpio_request(GPIO_SPEAKER, "SPEAKER"))
		printk(KERN_WARNING "vpr200: Couldn't get SPEAKER gpio\n");
	else
		gpio_direction_output(GPIO_SPEAKER, 0);

	imx35_add_imx_uart0(NULL);
	imx35_add_imx_uart2(NULL);

	mxc_register_device(&mx3_ipu, &mx3_ipu_data);
	mxc_register_device(&mx3_fb, &mx3fb_pdata);

	imx35_add_fsl_usb2_udc(&otg_device_pdata);
	imx35_add_mxc_ehci_hs(&usb_host_pdata);

	imx35_add_mxc_nand(&vpr200_nand_board_info);
	imx35_add_sdhci_esdhc_imx(0, &sd1_pdata);

#if defined(CONFIG_SND_SOC_VPR200_PCM1774)
	/* SSI unit master I2S codec connected to SSI_AUD4 */
	mxc_audmux_v2_configure_port(0,
			MXC_AUDMUX_V2_PTCR_SYN |
			MXC_AUDMUX_V2_PTCR_TFSDIR |
			MXC_AUDMUX_V2_PTCR_TFSEL(3) |
			MXC_AUDMUX_V2_PTCR_TCLKDIR |
			MXC_AUDMUX_V2_PTCR_TCSEL(3),
			MXC_AUDMUX_V2_PDCR_RXDSEL(3)
	);
	mxc_audmux_v2_configure_port(3,
			MXC_AUDMUX_V2_PTCR_SYN,
			MXC_AUDMUX_V2_PDCR_RXDSEL(0)
	);
#endif
	imx35_add_imx_ssi(0, &vpr200_ssi_pdata);

	i2c_register_board_info(0, vpr200_i2c_devices,
			ARRAY_SIZE(vpr200_i2c_devices));

	i2c_register_board_info(1, vpr200_bus1_devices,
			ARRAY_SIZE(vpr200_bus1_devices));

	imx35_add_imx_i2c0(&vpr200_i2c0_data);
	imx35_add_imx_i2c1(&vpr200_i2c0_data);

	mxc_init_pcm1774();
}

static void __init vpr200_timer_init(void)
{
	mx35_clocks_init();
}

struct sys_timer vpr200_timer = {
	.init	= vpr200_timer_init,
};

MACHINE_START(VPR200, "VPR200")
	/* Maintainer: Creative Product Design */
	.map_io = mx35_map_io,
	.init_early = imx35_init_early,
	.init_irq = mx35_init_irq,
	.timer = &vpr200_timer,
	.init_machine = vpr200_board_init,
MACHINE_END
