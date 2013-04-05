/*
 * Copyright (C) 2012 Synopsys, Inc. (www.synopsys.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>

#include <asm-generic/sizes.h>

#include <asm/irq.h>
#include <plat/irq.h>
#include <plat/memmap.h>
#include <plat/am_regs.h>
#include <plat/usbclock.h>
#include <plat/nand.h>
#include <plat/card_io.h>
#include <plat/pinmux.h>

/* ------------------------------------------------------------------------- */
#ifdef CONFIG_USB_DWCOTG
static struct resource dwc_otg_a_resources[] = {
	[0] = 	{
			.start	= IO_USB_A_BASE,
			.end	= IO_USB_A_BASE + SZ_128K - 1,
			.flags	= IORESOURCE_MEM,
		},
	[1] =	{
			.start	= AM_ISA_GEN_IRQ(30),
			.end	= AM_ISA_GEN_IRQ(30),
			.flags	= IORESOURCE_IRQ,
		},
};

static struct resource dwc_otg_b_resources[] = {
	[0] = 	{
			.start	= IO_USB_B_BASE,
			.end	= IO_USB_B_BASE + SZ_128K - 1,
			.flags	= IORESOURCE_MEM,
		},
	[1] =	{
			.start	= AM_ISA_GEN_IRQ(31),
			.end	= AM_ISA_GEN_IRQ(31),
			.flags	= IORESOURCE_IRQ,
		},
};

static u64 dwc_otg_dmamask = DMA_BIT_MASK(32);

static struct platform_device dwc_otg_a_dev = {
	.name		= "dwc_otg",
	.id		= 0,
	.resource	= dwc_otg_a_resources,
	.num_resources	= ARRAY_SIZE(dwc_otg_a_resources),
	.dev		= {	
				.dma_mask		= &dwc_otg_dmamask,
				.coherent_dma_mask	= DMA_BIT_MASK(32),
			},
};

static struct platform_device dwc_otg_b_dev = {
	.name		= "dwc_otg",
	.id		= 1,
	.resource	= dwc_otg_b_resources,
	.num_resources	= ARRAY_SIZE(dwc_otg_b_resources),
	.dev		= {	
				.dma_mask		= &dwc_otg_dmamask,
				.coherent_dma_mask	= DMA_BIT_MASK(32),
			},
};
#endif /* CONFIG_USB_DWCOTG */
/* ------------------------------------------------------------------------- */
#define OSD1_ADDR_START	0x90000000
#define OSD1_ADDR_END	0x90ffffff
#define OSD2_ADDR_START	0x91000000
#define OSD2_ADDR_END	0x91ffffff

static struct resource apollofb_device_resources[] = {
    [0] = {
        .start = AM_ISA_GEN_IRQ(INT_VIU_VSYNC),
        .end   = AM_ISA_GEN_IRQ(INT_VIU_VSYNC),
        .flags = IORESOURCE_IRQ,
    },
    [1] = {
        .start = OSD1_ADDR_START,
        .end   = OSD1_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
    [2] = { //for osd2
        .start = OSD2_ADDR_START,
        .end   = OSD2_ADDR_END,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device apollofb_device = {
    .name       = "amlfb",
    .id         = 0,
    .num_resources = ARRAY_SIZE(apollofb_device_resources),
    .resource      = apollofb_device_resources,
};

static struct resource vout_device_resources[] = {
    [0] = {
        .start = 0,
        .end   = 0,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device vout_device = {
    .name       = "amlvout",
    .id         = 0,
    .num_resources = ARRAY_SIZE(vout_device_resources),
    .resource      = vout_device_resources,
};


#if defined(CONFIG_CARDREADER)

static struct resource amlogic_card_resource[]  = {
    [0] = {
        .start = 0x1200230,   //physical address
        .end   = 0x120024c,
        .flags = 0x200,
    }
};


static struct aml_card_info  amlogic_card_info[] = {
    [0] = {
        .name = "sd_card",
        .work_mode = CARD_HW_MODE,
        .io_pad_type = SDIO_GPIOB_0_5,
        .card_ins_en_reg = EGPIO_GPIOD_ENABLE,
        .card_ins_en_mask = PREG_IO_18_MASK,
        .card_ins_input_reg = EGPIO_GPIOD_INPUT,
        .card_ins_input_mask = PREG_IO_18_MASK,
        .card_power_en_reg = 0,
        .card_power_en_mask = 0,
        .card_power_output_reg = 0,
        .card_power_output_mask = 0,
        .card_power_en_lev = 0,
        .card_wp_en_reg = EGPIO_GPIOD_ENABLE,
        .card_wp_en_mask = PREG_IO_19_MASK,
        .card_wp_input_reg = EGPIO_GPIOD_INPUT,
        .card_wp_input_mask = PREG_IO_19_MASK,
        .card_extern_init = 0,
    },
};

static struct aml_card_platform amlogic_card_platform = {
    .card_num = ARRAY_SIZE(amlogic_card_info),
    .card_info = amlogic_card_info,
};

static struct platform_device amlogic_card_device = {
    .name = "AMLOGIC_CARD",
    .id    = -1,
    .num_resources = ARRAY_SIZE(amlogic_card_resource),
    .resource = amlogic_card_resource,
    .dev = {
        .platform_data = &amlogic_card_platform,
    },
};
#endif

#if defined(CONFIG_AM_NAND)
static struct mtd_partition multi_partition_info[] = {
    {
        .name = "environment",
        .offset = 8 * 1024 * 1024,
        .size = 40 * 1024 * 1024,
    },
    {
        .name = "logo",
        .offset = 48 * 1024 * 1024,
        .size = 16 * 1024 * 1024,
    },
    {
        .name = "recovery",
        .offset = 64 * 1024 * 1024,
        .size = 16 * 1024 * 1024,
    },
    {
        .name = "uImage",
        .offset = 80 * 1024 * 1024,
        .size = 16 * 1024 * 1024,
    },
    {
        .name = "system",
        .offset = 96 * 1024 * 1024,
        .size = 256 * 1024 * 1024,
    },
    {
        .name = "cache",
        .offset = 352 * 1024 * 1024,
        .size = 40 * 1024 * 1024,
    },
    {
        .name = "userdata",
        .offset = 392 * 1024 * 1024,
        .size = 512 * 1024 * 1024,
    },
    {
        .name = "NFTL_Part",
        .offset = ((392 + 512) * 1024 * 1024),
        .size = (((uint64_t)0x40000000 - (392 + 512) * 1024 * 1024)),
    },
};


static struct aml_nand_platform aml_nand_mid_platform[] = {
    /*{
        .name = NAND_BOOT_NAME,
        .chip_enable_pad = AML_NAND_CE0,
        .ready_busy_pad = AML_NAND_CE0,
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .options = (NAND_TIMING_MODE5 | NAND_ECC_BCH60_MODE),
            },
        },
        .rbpin_mode=1,
        .short_pgsz=384,
        .ran_mode=0,
        .T_REA = 20,
        .T_RHOH = 15,
    },*/
    {
        .name = NAND_MULTI_NAME,
        .chip_enable_pad = (AML_NAND_CE0) ,  //| (AML_NAND_CE1 << 4) | (AML_NAND_CE2 << 8) | (AML_NAND_CE3 << 12)),
        .ready_busy_pad = (AML_NAND_CE0) ,  //| (AML_NAND_CE0 << 4) | (AML_NAND_CE1 << 8) | (AML_NAND_CE1 << 12)),
        .platform_nand_data = {
            .chip =  {
                .nr_chips = 1,
                .nr_partitions = ARRAY_SIZE(multi_partition_info),
                .partitions = multi_partition_info,
		.options = (NAND_TIMING_MODE5 | NAND_ECC_BCH30_MODE),
            },
        },
        .rbpin_mode = 1,
        .short_pgsz = 0,
        .ran_mode = 0,
        .T_REA = 20,
        .T_RHOH = 15,
    }
};

struct aml_nand_device aml_nand_mid_device = {
    .aml_nand_platform = aml_nand_mid_platform,
    .dev_num = ARRAY_SIZE(aml_nand_mid_platform),
};

static struct resource aml_nand_resources[] = {
    {
        .start = 0xc1108600,
        .end = 0xc1108624,
        .flags = IORESOURCE_MEM,
    },
};

static struct platform_device a3_nand_device = {
    .name = "aml_a3_nand",
    .id = 0,
    .num_resources = ARRAY_SIZE(aml_nand_resources),
    .resource = aml_nand_resources,
    .dev = {
        .platform_data = &aml_nand_mid_device,
    },
};
#endif


#ifdef CONFIG_I2C_AML
#include <linux/i2c.h>
#include <linux/i2c-aml.h>

static struct aml_i2c_platform aml_i2c_plat = {
    .wait_count     = 1000,
    .wait_ack_interval  = 5,
    .wait_read_interval = 5,
    .wait_xfer_interval = 50,
    .master_no      = AML_I2C_MASTER_B,
    .use_pio            = 0,
    .master_i2c_speed   = AML_I2C_SPPED_100K,

    .master_pinmux = {
        .scl_reg    = A3_I2C_MASTER_B_GPIOA_5_REG,
        .scl_bit    = A3_I2C_MASTER_B_GPIOA_5_BIT,
        .sda_reg    = A3_I2C_MASTER_B_GPIOA_4_REG,
        .sda_bit    = A3_I2C_MASTER_B_GPIOA_4_BIT,
    }
};

static struct resource aml_i2c_resource[] = {
    [0] = {/*master a*/
        .start =    MESON_I2C_MASTER_A_START,
        .end   =    MESON_I2C_MASTER_A_END,
        .flags =    IORESOURCE_MEM,
    },
    [1] = {/*master b*/
        .start =    MESON_I2C_MASTER_B_START,
        .end   =    MESON_I2C_MASTER_B_END,
        .flags =    IORESOURCE_MEM,
    },
    [2] = {/*slave*/
        .start =    MESON_I2C_SLAVE_START,
        .end   =    MESON_I2C_SLAVE_END,
        .flags =    IORESOURCE_MEM,
    },
};

static struct platform_device aml_i2c_device = {
    .name         = "aml-i2c",
    .id       = -1,
    .num_resources    = ARRAY_SIZE(aml_i2c_resource),
    .resource     = aml_i2c_resource,
    .dev = {
        .platform_data = &aml_i2c_plat,
    },
};

#endif


static struct platform_device *dw_platform_devices[] __initdata = {
#ifdef CONFIG_USB_DWCOTG

/* dwc_otg_a is for the USB mini a/b otg connector. As long as this feature
   is not used, we don't load this controller, as initialization takes ~530 ms
   at boot time. */
#ifdef CONFIG_USB_GADGET
	&dwc_otg_a_dev,
#endif
	&dwc_otg_b_dev,
#endif /* CONFIG_USB_DWCOTG */
	&vout_device,
	&apollofb_device,
#if defined(CONFIG_CARDREADER)
    &amlogic_card_device,
#endif
#ifdef CONFIG_AM_NAND
	&a3_nand_device,
#endif
#ifdef CONFIG_I2C_AML
	&aml_i2c_device,
#endif
};

static struct i2c_board_info __initdata aml_i2c_info[] = {
#ifdef CONFIG_EEPROM_AT24
    {I2C_BOARD_INFO("24c02",  0x50),},
    /*{I2C_BOARD_INFO("24c64",  0x50),},*/
#endif
#ifdef CONFIG_TOUCHSCREEN_TSC2007
    {
        I2C_BOARD_INFO("tsc2007", 0x48),
        .irq = INT_GPIO_0,
        .platform_data = (void *)&tsc2007_pdata,
    },
#endif
#ifdef CONFIG_SN7325
    {
        I2C_BOARD_INFO("sn7325", 0x59),
        .platform_data = (void *)&sn7325_pdata,
    },
#endif
#ifdef CONFIG_SND_AML_A3_WM8988
    {
        I2C_BOARD_INFO("wm8988", 0x1A),
        .platform_data = (void *)0,
    },
#endif
	{
		I2C_BOARD_INFO("gt811", 0x5D),
		//I2C_BOARD_INFO("gt811", 0x1c),
	},
};


int __init dw_platform_init(void)
{
#ifdef CONFIG_USB_DWCOTG
	set_usb_phy_clk(USB_PHY_CLOCK_SEL_XTAL_DIV2);
	set_usb_phy_id_mode(USB_PHY_PORT_A, USB_PHY_MODE_HW);
	set_usb_phy_id_mode(USB_PHY_PORT_B, USB_PHY_MODE_SW_HOST);
#endif

#ifdef CONFIG_I2C_AML
	clear_mio_mux(0, (1 << 5) |(1 << 6) |(1 << 7) |
			 (1 << 8) |(1 << 9) |(1 << 10));
	clear_mio_mux(6, (1 << 6) |(1 << 7));
	clear_mio_mux(7, (1 << 12) |(1 << 13));
	set_mio_mux(6, (1 << 8) |(1 << 9));
	clear_mio_mux(3, (1 << 4) |(1 << 12));

	i2c_register_board_info (1, aml_i2c_info, ARRAY_SIZE(aml_i2c_info));
#endif

	return platform_add_devices(dw_platform_devices,
		ARRAY_SIZE(dw_platform_devices));
}

arch_initcall(dw_platform_init);
