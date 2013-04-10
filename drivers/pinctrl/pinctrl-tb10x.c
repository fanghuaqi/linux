/*
 * Abilis Systems TB10x pin control driver
 *
 * Copyright (C) Abilis Systems 2012
 *
 * Author: Christian Ruppert <christian.ruppert@abilis.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#include <linux/stringify.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/machine.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl-tb10x.h>

#define TB10X_IOMUX_COMPATIBLE ("abilis,tb10x-iomux")

static const struct pinctrl_pin_desc tb10x_pins[] = {
	/* Port 1 */
	PINCTRL_PIN(0 +  0, "MICLK_S0"),
	PINCTRL_PIN(0 +  1, "MISTRT_S0"),
	PINCTRL_PIN(0 +  2, "MIVAL_S0"),
	PINCTRL_PIN(0 +  3, "MDI_S0"),
	PINCTRL_PIN(0 +  4, "GPIOA0"),
	PINCTRL_PIN(0 +  5, "GPIOA1"),
	PINCTRL_PIN(0 +  6, "GPIOA2"),
	PINCTRL_PIN(0 +  7, "MDI_S1"),
	PINCTRL_PIN(0 +  8, "MIVAL_S1"),
	PINCTRL_PIN(0 +  9, "MISTRT_S1"),
	PINCTRL_PIN(0 + 10, "MICLK_S1"),
	/* Port 2 */
	PINCTRL_PIN(16 +  0, "MICLK_S2"),
	PINCTRL_PIN(16 +  1, "MISTRT_S2"),
	PINCTRL_PIN(16 +  2, "MIVAL_S2"),
	PINCTRL_PIN(16 +  3, "MDI_S2"),
	PINCTRL_PIN(16 +  4, "GPIOC0"),
	PINCTRL_PIN(16 +  5, "GPIOC1"),
	PINCTRL_PIN(16 +  6, "GPIOC2"),
	PINCTRL_PIN(16 +  7, "MDI_S3"),
	PINCTRL_PIN(16 +  8, "MIVAL_S3"),
	PINCTRL_PIN(16 +  9, "MISTRT_S3"),
	PINCTRL_PIN(16 + 10, "MICLK_S3"),
	/* Port 3 */
	PINCTRL_PIN(32 +  0, "MICLK_S4"),
	PINCTRL_PIN(32 +  1, "MISTRT_S4"),
	PINCTRL_PIN(32 +  2, "MIVAL_S4"),
	PINCTRL_PIN(32 +  3, "MDI_S4"),
	PINCTRL_PIN(32 +  4, "GPIOE0"),
	PINCTRL_PIN(32 +  5, "GPIOE1"),
	PINCTRL_PIN(32 +  6, "GPIOE2"),
	PINCTRL_PIN(32 +  7, "MDI_S5"),
	PINCTRL_PIN(32 +  8, "MIVAL_S5"),
	PINCTRL_PIN(32 +  9, "MISTRT_S5"),
	PINCTRL_PIN(32 + 10, "MICLK_S5"),
	/* Port 4 */
	PINCTRL_PIN(48 +  0, "MICLK_S6"),
	PINCTRL_PIN(48 +  1, "MISTRT_S6"),
	PINCTRL_PIN(48 +  2, "MIVAL_S6"),
	PINCTRL_PIN(48 +  3, "MDI_S6"),
	PINCTRL_PIN(48 +  4, "GPIOG0"),
	PINCTRL_PIN(48 +  5, "GPIOG1"),
	PINCTRL_PIN(48 +  6, "GPIOG2"),
	PINCTRL_PIN(48 +  7, "MDI_S7"),
	PINCTRL_PIN(48 +  8, "MIVAL_S7"),
	PINCTRL_PIN(48 +  9, "MISTRT_S7"),
	PINCTRL_PIN(48 + 10, "MICLK_S7"),
	/* Port 6 */
	PINCTRL_PIN(64 + 0, "T_MOSTRT_S0"),
	PINCTRL_PIN(64 + 1, "T_MOVAL_S0"),
	PINCTRL_PIN(64 + 2, "T_MDO_S0"),
	PINCTRL_PIN(64 + 3, "T_MOSTRT_S1"),
	PINCTRL_PIN(64 + 4, "T_MOVAL_S1"),
	PINCTRL_PIN(64 + 5, "T_MDO_S1"),
	PINCTRL_PIN(64 + 6, "T_MOSTRT_S2"),
	PINCTRL_PIN(64 + 7, "T_MOVAL_S2"),
	PINCTRL_PIN(64 + 8, "T_MDO_S2"),
	PINCTRL_PIN(64 + 9, "T_MOSTRT_S3"),
	/* Port 7 */
	PINCTRL_PIN(80 + 0, "UART0_TXD"),
	PINCTRL_PIN(80 + 1, "UART0_RXD"),
	PINCTRL_PIN(80 + 2, "UART0_CTS"),
	PINCTRL_PIN(80 + 3, "UART0_RTS"),
	PINCTRL_PIN(80 + 4, "UART1_TXD"),
	PINCTRL_PIN(80 + 5, "UART1_RXD"),
	PINCTRL_PIN(80 + 6, "UART1_CTS"),
	PINCTRL_PIN(80 + 7, "UART1_RTS"),
	/* Port 8 */
	PINCTRL_PIN(96 + 0, "SPI3_CLK"),
	PINCTRL_PIN(96 + 1, "SPI3_MISO"),
	PINCTRL_PIN(96 + 2, "SPI3_MOSI"),
	PINCTRL_PIN(96 + 3, "SPI3_SSN"),
	/* Port 9 */
	PINCTRL_PIN(112 + 0, "SPI1_CLK"),
	PINCTRL_PIN(112 + 1, "SPI1_MISO"),
	PINCTRL_PIN(112 + 2, "SPI1_MOSI"),
	PINCTRL_PIN(112 + 3, "SPI1_SSN0"),
	PINCTRL_PIN(112 + 4, "SPI1_SSN1"),
	/* Port 5 */
	PINCTRL_PIN(128 +  0, "PC_CE1N"),
	PINCTRL_PIN(128 +  1, "PC_CE2N"),
	PINCTRL_PIN(128 +  2, "PC_REGN"),
	PINCTRL_PIN(128 +  3, "PC_INPACKN"),
	PINCTRL_PIN(128 +  4, "PC_OEN"),
	PINCTRL_PIN(128 +  5, "PC_WEN"),
	PINCTRL_PIN(128 +  6, "PC_IORDN"),
	PINCTRL_PIN(128 +  7, "PC_IOWRN"),
	PINCTRL_PIN(128 +  8, "PC_RDYIRQN"),
	PINCTRL_PIN(128 +  9, "PC_WAITN"),
	PINCTRL_PIN(128 + 10, "PC_A0"),
	PINCTRL_PIN(128 + 11, "PC_A1"),
	PINCTRL_PIN(128 + 12, "PC_A2"),
	PINCTRL_PIN(128 + 13, "PC_A3"),
	PINCTRL_PIN(128 + 14, "PC_A4"),
	PINCTRL_PIN(128 + 15, "PC_A5"),
	PINCTRL_PIN(128 + 16, "PC_A6"),
	PINCTRL_PIN(128 + 17, "PC_A7"),
	PINCTRL_PIN(128 + 18, "PC_A8"),
	PINCTRL_PIN(128 + 19, "PC_A9"),
	PINCTRL_PIN(128 + 20, "PC_A10"),
	PINCTRL_PIN(128 + 21, "PC_A11"),
	PINCTRL_PIN(128 + 22, "PC_A12"),
	PINCTRL_PIN(128 + 23, "PC_A13"),
	PINCTRL_PIN(128 + 24, "PC_A14"),
	PINCTRL_PIN(128 + 25, "PC_D0"),
	PINCTRL_PIN(128 + 26, "PC_D1"),
	PINCTRL_PIN(128 + 27, "PC_D2"),
	PINCTRL_PIN(128 + 28, "PC_D3"),
	PINCTRL_PIN(128 + 29, "PC_D4"),
	PINCTRL_PIN(128 + 30, "PC_D5"),
	PINCTRL_PIN(128 + 31, "PC_D6"),
	PINCTRL_PIN(128 + 32, "PC_D7"),
	PINCTRL_PIN(128 + 33, "PC_MOSTRT"),
	PINCTRL_PIN(128 + 34, "PC_MOVAL"),
	PINCTRL_PIN(128 + 35, "PC_MDO0"),
	PINCTRL_PIN(128 + 36, "PC_MDO1"),
	PINCTRL_PIN(128 + 37, "PC_MDO2"),
	PINCTRL_PIN(128 + 38, "PC_MDO3"),
	PINCTRL_PIN(128 + 39, "PC_MDO4"),
	PINCTRL_PIN(128 + 40, "PC_MDO5"),
	PINCTRL_PIN(128 + 41, "PC_MDO6"),
	PINCTRL_PIN(128 + 42, "PC_MDO7"),
	PINCTRL_PIN(128 + 43, "PC_MISTRT"),
	PINCTRL_PIN(128 + 44, "PC_MIVAL"),
	PINCTRL_PIN(128 + 45, "PC_MDI0"),
	PINCTRL_PIN(128 + 46, "PC_MDI1"),
	PINCTRL_PIN(128 + 47, "PC_MDI2"),
	PINCTRL_PIN(128 + 48, "PC_MDI3"),
	PINCTRL_PIN(128 + 49, "PC_MDI4"),
	PINCTRL_PIN(128 + 50, "PC_MDI5"),
	PINCTRL_PIN(128 + 51, "PC_MDI6"),
	PINCTRL_PIN(128 + 52, "PC_MDI7"),
	PINCTRL_PIN(128 + 53, "PC_MICLK"),
	/* Unmuxed GPIOs */
	PINCTRL_PIN(256 +  0, "GPIOB0"),
	PINCTRL_PIN(256 +  1, "GPIOB1"),

	PINCTRL_PIN(256 +  2, "GPIOD0"),
	PINCTRL_PIN(256 +  3, "GPIOD1"),

	PINCTRL_PIN(256 +  4, "GPIOF0"),
	PINCTRL_PIN(256 +  5, "GPIOF1"),

	PINCTRL_PIN(256 +  6, "GPIOH0"),
	PINCTRL_PIN(256 +  7, "GPIOH1"),

	PINCTRL_PIN(256 +  8, "GPIOI0"),
	PINCTRL_PIN(256 +  9, "GPIOI1"),
	PINCTRL_PIN(256 + 10, "GPIOI2"),
	PINCTRL_PIN(256 + 11, "GPIOI3"),
	PINCTRL_PIN(256 + 12, "GPIOI4"),
	PINCTRL_PIN(256 + 13, "GPIOI5"),
	PINCTRL_PIN(256 + 14, "GPIOI6"),
	PINCTRL_PIN(256 + 15, "GPIOI7"),
	PINCTRL_PIN(256 + 16, "GPIOI8"),
	PINCTRL_PIN(256 + 17, "GPIOI9"),
	PINCTRL_PIN(256 + 18, "GPIOI10"),
	PINCTRL_PIN(256 + 19, "GPIOI11"),

	PINCTRL_PIN(256 + 20, "GPION0"),
	PINCTRL_PIN(256 + 21, "GPION1"),
	PINCTRL_PIN(256 + 22, "GPION2"),
	PINCTRL_PIN(256 + 23, "GPION3"),
#define MAX_PIN (256 + 24)
	PINCTRL_PIN(MAX_PIN,  "GPION4"),
};


/* Port 1 */
static const unsigned mis0_pins[]  = { 0,  1,  2,  3};
static const unsigned gpioa_pins[] = { 4,  5,  6};
static const unsigned mis1_pins[]  = { 7,  8,  9, 10};
static const unsigned mip1_pins[]  = { 0,  1,  2,  3,  4,  5,
					6,  7,  8,  9, 10};

/* Port 2 */
static const unsigned mis2_pins[]  = {16, 17, 18, 19};
static const unsigned gpioc_pins[] = {20, 21, 22};
static const unsigned mis3_pins[]  = {23, 24, 25, 26};
static const unsigned mip3_pins[]  = {16, 17, 18, 19, 20, 21,
					22, 23, 24, 25, 26};

/* Port 3 */
static const unsigned mis4_pins[]  = {32, 33, 34, 35};
static const unsigned gpioe_pins[] = {36, 37, 38};
static const unsigned mis5_pins[]  = {39, 40, 41, 42};
static const unsigned mip5_pins[]  = {32, 33, 34, 35, 36, 37,
					38, 39, 40, 41, 42};

/* Port 4 */
static const unsigned mis6_pins[]  = {48, 49, 50, 51};
static const unsigned gpiog_pins[] = {52, 53, 54};
static const unsigned mis7_pins[]  = {55, 56, 57, 58};
static const unsigned mip7_pins[]  = {48, 49, 50, 51, 52, 53,
					54, 55, 56, 57, 58};

/* Port 6 */
static const unsigned mop_pins[] = {64, 65, 66, 67, 68,
					69, 70, 71, 72, 73};
static const unsigned mos0_pins[] = {64, 65, 66};
static const unsigned mos1_pins[] = {67, 68, 69};
static const unsigned mos2_pins[] = {70, 71, 72};
static const unsigned mos3_pins[] = {73};

/* Port 7 */
static const unsigned uart0_pins[] = {80, 81, 82, 83};
static const unsigned uart1_pins[] = {84, 85, 86, 87};
static const unsigned gpiol_pins[] = {80, 81, 82, 83};
static const unsigned gpiom_pins[] = {84, 85, 86, 87};

/* Port 8 */
static const unsigned spi3_pins[] = {96, 97, 98, 99};
static const unsigned jtag_pins[] = {96, 97, 98, 99};

/* Port 9 */
static const unsigned spi1_pins[] = {112, 113, 114, 115, 116};
static const unsigned gpion_pins[] = {112, 113, 114, 115, 116};

/* Port 5 */
static const unsigned gpioj_pins[] = {128, 129, 130, 131, 132, 133, 134,
					135, 136, 137, 138, 139, 140, 141,
					142, 143, 144, 145, 146, 147, 148,
					149, 150, 151, 152, 153, 154, 155,
					156, 157, 158, 159};
static const unsigned gpiok_pins[] = {160, 161, 162, 163, 164, 165, 166, 167,
					168, 169, 170, 171, 172, 173, 174,
					175, 176, 177, 178, 179, 180, 181};
static const unsigned ciplus_pins[] = {128, 129, 130, 131, 132, 133, 134,
					135, 136, 137, 138, 139, 140, 141, 142,
					143, 144, 145, 146, 147, 148, 149, 150,
					151, 152, 153, 154, 155, 156, 157, 158,
					159, 160, 161, 162, 163, 164, 165, 166,
					167, 168, 169, 170, 171, 172, 173, 174,
					175, 176, 177, 178, 179, 180, 181};
static const unsigned mcard_pins[] = {131, 138, 139, 140, 150, 151, 161,
					163, 164, 165, 166, 167, 168, 169, 170,
					171, 173, 174, 175, 176, 177, 178, 179,
					180, 181};
static const unsigned stc0_pins[] = {162, 163, 164, 165, 166, 167, 168};
static const unsigned stc1_pins[] = {153, 154, 155, 156, 157, 158, 172};

/* Unmuxed GPIOs */
static const unsigned gpiob_pins[] = {256, 257};
static const unsigned gpiod_pins[] = {258, 259};
static const unsigned gpiof_pins[] = {260, 261};
static const unsigned gpioh_pins[] = {262, 263};
static const unsigned gpioi_pins[] = {264, 265, 266, 267, 268, 269, 270,
					271, 272, 273, 274, 275};


struct tb10x_pinfuncgrp {
	const char *name;
	const unsigned int *pins;
	const unsigned int pincnt;
	const int port;
	const unsigned int mode;
	const int isgpio;
	struct tb10x_pinctrl_state *pctl;
};
#define DEFPINFUNCGRP(NAME, PORT, MODE, ISGPIO) { \
		.name = __stringify(NAME), \
		.pins = NAME, .pincnt = ARRAY_SIZE(NAME), \
		.port = (PORT), .mode = (MODE), .pctl = NULL, \
		.isgpio = (ISGPIO), \
	}
static struct tb10x_pinfuncgrp tb10x_pingroups[] = {
	DEFPINFUNCGRP(mis0_pins,   0, 0, 0),
	DEFPINFUNCGRP(gpioa_pins,  0, 0, 1),
	DEFPINFUNCGRP(mis1_pins,   0, 0, 0),
	DEFPINFUNCGRP(mip1_pins,   0, 1, 0),
	DEFPINFUNCGRP(mis2_pins,   1, 0, 0),
	DEFPINFUNCGRP(gpioc_pins,  1, 0, 1),
	DEFPINFUNCGRP(mis3_pins,   1, 0, 0),
	DEFPINFUNCGRP(mip3_pins,   1, 1, 0),
	DEFPINFUNCGRP(mis4_pins,   2, 0, 0),
	DEFPINFUNCGRP(gpioe_pins,  2, 0, 1),
	DEFPINFUNCGRP(mis5_pins,   2, 0, 0),
	DEFPINFUNCGRP(mip5_pins,   2, 1, 0),
	DEFPINFUNCGRP(mis6_pins,   3, 0, 0),
	DEFPINFUNCGRP(gpiog_pins,  3, 0, 1),
	DEFPINFUNCGRP(mis7_pins,   3, 0, 0),
	DEFPINFUNCGRP(mip7_pins,   3, 1, 0),
	DEFPINFUNCGRP(gpioj_pins,  4, 0, 1),
	DEFPINFUNCGRP(gpiok_pins,  4, 0, 1),
	DEFPINFUNCGRP(ciplus_pins, 4, 1, 0),
	DEFPINFUNCGRP(mcard_pins,  4, 2, 0),
	DEFPINFUNCGRP(stc0_pins,   4, 3, 0),
	DEFPINFUNCGRP(stc1_pins,   4, 3, 0),
	DEFPINFUNCGRP(mop_pins,    5, 0, 0),
	DEFPINFUNCGRP(mos0_pins,   5, 1, 0),
	DEFPINFUNCGRP(mos1_pins,   5, 1, 0),
	DEFPINFUNCGRP(mos2_pins,   5, 1, 0),
	DEFPINFUNCGRP(mos3_pins,   5, 1, 0),
	DEFPINFUNCGRP(uart0_pins,  6, 0, 0),
	DEFPINFUNCGRP(uart1_pins,  6, 0, 0),
	DEFPINFUNCGRP(gpiol_pins,  6, 1, 1),
	DEFPINFUNCGRP(gpiom_pins,  6, 1, 1),
	DEFPINFUNCGRP(spi3_pins,   7, 0, 0),
	DEFPINFUNCGRP(jtag_pins,   7, 1, 0),
	DEFPINFUNCGRP(spi1_pins,   8, 0, 0),
	DEFPINFUNCGRP(gpion_pins,  8, 1, 1),
	DEFPINFUNCGRP(gpiob_pins, -1, 0, 1),
	DEFPINFUNCGRP(gpiod_pins, -1, 0, 1),
	DEFPINFUNCGRP(gpiof_pins, -1, 0, 1),
	DEFPINFUNCGRP(gpioh_pins, -1, 0, 1),
	DEFPINFUNCGRP(gpioi_pins, -1, 0, 1),
};
#undef DEFPINFUNCGRP

struct tb10x_of_pinfunc {
	const char *name;
	const char *group;
};

#define TB10X_PORTS (9)

struct tb10x_pinctrl_state {
	struct pinctrl_dev *pctl;
	void *iobase;
	const struct tb10x_pinfuncgrp *pingroups;
	unsigned int pinfuncgrpcnt;
	struct tb10x_of_pinfunc *pinfuncs;
	unsigned int pinfuncnt;
	struct mutex mutex;
	struct {
		unsigned int mode;
		unsigned int count;
	} ports[TB10X_PORTS];
	DECLARE_BITMAP(gpios, MAX_PIN + 1);
};

static struct tb10x_pinfuncgrp *tb10x_get_pinfuncgrp(const char *name)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tb10x_pingroups); i++)
		if (!strcmp(name, tb10x_pingroups[i].name))
			return &tb10x_pingroups[i];

	return ERR_PTR(-ENODEV);
}

struct tb10x_pinfuncgrp *tb10x_prepare_gpio_range(struct device_node *dn,
						struct pinctrl_gpio_range *gr)
{
	const char *name;
	int ret;
	struct tb10x_pinfuncgrp *pfg;

	ret = of_property_read_string(dn, "pingrp", &name);
	if (ret)
		return ERR_PTR(ret);

	pfg = tb10x_get_pinfuncgrp(name);

	if (!IS_ERR(pfg)) {
		if (!pfg->isgpio)
			return ERR_PTR(-EINVAL);

		if (!pfg->pctl)
			return ERR_PTR(-EPROBE_DEFER);

		gr->pin_base = pfg->pins[0];
		gr->npins    = pfg->pincnt;
	}

	return pfg;
}
EXPORT_SYMBOL(tb10x_prepare_gpio_range);

void tb10x_setup_gpio_range(struct tb10x_pinfuncgrp *pfg,
					struct pinctrl_gpio_range *gr)
{
	pinctrl_add_gpio_range(pfg->pctl->pctl, gr);
}
EXPORT_SYMBOL(tb10x_setup_gpio_range);

static inline void tb10x_pinctrl_set_config(struct tb10x_pinctrl_state *state,
				unsigned int port, unsigned int mode)
{
	u32 pcfg;

	if (state->ports[port].count)
		return;

	state->ports[port].mode = mode;

	pcfg = ioread32(state->iobase) & ~(0x3 << (2 * port));
	iowrite32(pcfg | ((mode & 0x3) << (2 * port)), state->iobase);
}

static inline unsigned int tb10x_pinctrl_get_config(
				struct tb10x_pinctrl_state *state,
				unsigned int port)
{
	return (ioread32(state->iobase) >> (2 * port)) & 0x3;
}

static int tb10x_get_groups_count(struct pinctrl_dev *pctl)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	return state->pinfuncgrpcnt;
}

static const char *tb10x_get_group_name(struct pinctrl_dev *pctl, unsigned n)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	return state->pingroups[n].name;
}

static int tb10x_get_group_pins(struct pinctrl_dev *pctl, unsigned n,
				unsigned const **pins,
				unsigned * const num_pins)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	*pins = state->pingroups[n].pins;
	*num_pins = state->pingroups[n].pincnt;
	return 0;
}

static int tb10x_dt_node_to_map(struct pinctrl_dev *pctl,
				struct device_node *np_config,
				struct pinctrl_map **map, unsigned *num_maps)
{
	struct pinctrl_map *m;
	const char *string;

	if (of_property_read_string(np_config, "pingrp", &string)) {
		pr_err("%s: No pingrp property in device tree.\n",
			np_config->full_name);
		return -EINVAL;
	}

	m = kzalloc(sizeof(struct pinctrl_map), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->type = PIN_MAP_TYPE_MUX_GROUP;
	m->data.mux.group = string;
	m->data.mux.function = np_config->name;
	*map = m;
	*num_maps = 1;
	return 0;
}

static void tb10x_dt_free_map(struct pinctrl_dev *pctldev,
				struct pinctrl_map *map, unsigned num_maps)
{
	kfree(map);
}

static struct pinctrl_ops tb10x_pinctrl_ops = {
	.get_groups_count = tb10x_get_groups_count,
	.get_group_name   = tb10x_get_group_name,
	.get_group_pins   = tb10x_get_group_pins,
	.dt_node_to_map   = tb10x_dt_node_to_map,
	.dt_free_map      = tb10x_dt_free_map,
};

static int tb10x_get_functions_count(struct pinctrl_dev *pctl)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	return state->pinfuncnt;
}

static const char *tb10x_get_function_name(struct pinctrl_dev *pctl,
					unsigned n)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	return state->pinfuncs[n].name;
}

static int tb10x_get_function_groups(struct pinctrl_dev *pctl,
				unsigned n, const char * const **groups,
				unsigned * const num_groups)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	*groups = &state->pinfuncs[n].group;
	*num_groups = 1;
	return 0;
}

static int tb10x_gpio_request_enable(struct pinctrl_dev *pctl,
					struct pinctrl_gpio_range *range,
					unsigned pin)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	int muxport = -1;
	int muxmode = -1;
	int i;

	mutex_lock(&state->mutex);

	for (i = 0; i < ARRAY_SIZE(tb10x_pingroups); i++) {
		struct tb10x_pinfuncgrp *pfg = &tb10x_pingroups[i];
		unsigned int port = pfg->port;
		unsigned int mode = pfg->mode;
		int j;

		if (port < 0)
			continue;

		if (pfg->isgpio) {
			muxport = port;
			muxmode = mode;
			continue;
		}

		for (j = 0; j < pfg->pincnt; j++) {
			if (pin == pfg->pins[j]) {
				if (state->ports[port].count
					&& (state->ports[port].mode == mode)) {
					mutex_unlock(&state->mutex);
					return -EBUSY;
				}
				break;
			}
		}
	}

	set_bit(pin, state->gpios);

	/* don't need to check for count == 0 because set_config is a NOP
	 * if count != 0
	 */
	if (muxport >= 0)
		tb10x_pinctrl_set_config(state, muxport, muxmode);

	mutex_unlock(&state->mutex);

	return 0;
}

static void tb10x_gpio_disable_free(struct pinctrl_dev *pctl,
					struct pinctrl_gpio_range *range,
					unsigned pin)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);

	mutex_lock(&state->mutex);

	clear_bit(pin, state->gpios);

	mutex_unlock(&state->mutex);
}

static int tb10x_pctl_enable(struct pinctrl_dev *pctl,
			unsigned func_selector, unsigned group_selector)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	const struct tb10x_pinfuncgrp *grp = &state->pingroups[group_selector];
	int i;

	if (grp->port < 0)
		return 0;

	mutex_lock(&state->mutex);

	if (state->ports[grp->port].count
			&& (state->ports[grp->port].mode != grp->mode)) {
		mutex_unlock(&state->mutex);
		return -EBUSY;
	}

	for (i = 0; i < grp->pincnt; i++)
		if (test_bit(grp->pins[i], state->gpios)) {
			mutex_unlock(&state->mutex);
			return -EBUSY;
		}

	tb10x_pinctrl_set_config(state, grp->port, grp->mode);

	state->ports[grp->port].count++;

	mutex_unlock(&state->mutex);

	return 0;
}

static void tb10x_pctl_disable(struct pinctrl_dev *pctl,
			unsigned func_selector, unsigned group_selector)
{
	struct tb10x_pinctrl_state *state = pinctrl_dev_get_drvdata(pctl);
	const struct tb10x_pinfuncgrp *grp = &state->pingroups[group_selector];

	if (grp->port < 0)
		return;

	mutex_lock(&state->mutex);

	state->ports[grp->port].count--;

	mutex_unlock(&state->mutex);
}

static struct pinmux_ops tb10x_pinmux_ops = {
	.get_functions_count = tb10x_get_functions_count,
	.get_function_name = tb10x_get_function_name,
	.get_function_groups = tb10x_get_function_groups,
	.gpio_request_enable = tb10x_gpio_request_enable,
	.gpio_disable_free = tb10x_gpio_disable_free,
	.enable = tb10x_pctl_enable,
	.disable = tb10x_pctl_disable,
};

static struct pinctrl_desc tb10x_pindesc = {
	.name = "TB10x",
	.pins = tb10x_pins,
	.npins = ARRAY_SIZE(tb10x_pins),
	.owner = THIS_MODULE,
	.pctlops = &tb10x_pinctrl_ops,
	.pmxops  = &tb10x_pinmux_ops,
};

static int tb10x_pinctrl_probe(struct platform_device *pdev)
{
	int ret = -EINVAL;
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *child;
	struct tb10x_pinctrl_state *state;
	int i;

	if (!of_node) {
		dev_err(&pdev->dev, "No device tree node found.\n");
		return -EINVAL;
	}

	if (!mem) {
		dev_err(&pdev->dev, "No memory resource defined.\n");
		return -EINVAL;
	}

	state = kzalloc(sizeof(struct tb10x_pinctrl_state) +
			of_get_child_count(of_node)
			* sizeof(struct tb10x_of_pinfunc),
			GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	platform_set_drvdata(pdev, state);
	state->pinfuncs = (struct tb10x_of_pinfunc *)(state + 1);
	mutex_init(&state->mutex);

	if (!request_mem_region(mem->start, resource_size(mem), pdev->name)) {
		dev_err(&pdev->dev, "Request register region failed.\n");
		ret = -EBUSY;
		goto request_mem_fail;
	}

	state->iobase = ioremap(mem->start, resource_size(mem));
	if (!state->iobase) {
		dev_err(&pdev->dev, "Request register region failed.\n");
		ret = -EBUSY;
		goto ioremap_fail;
	}

	state->pingroups = tb10x_pingroups;
	state->pinfuncgrpcnt = ARRAY_SIZE(tb10x_pingroups);

	for (i = 0; i < TB10X_PORTS; i++)
		state->ports[i].mode = tb10x_pinctrl_get_config(state, i);

	for_each_child_of_node(of_node, child) {
		const char *name;

		if (!of_property_read_string(child, "pingrp", &name)) {
			struct tb10x_pinfuncgrp *pfg;

			pfg = tb10x_get_pinfuncgrp(name);

			if (IS_ERR(pfg) || pfg->pctl) {
				dev_err(&pdev->dev,
					"Could not allocate pin group %s.\n",
					name);
				continue;
			}
			pfg->pctl = state;
			state->pinfuncs[state->pinfuncnt].name = child->name;
			state->pinfuncs[state->pinfuncnt].group = name;
			state->pinfuncnt++;
		}
	}

	state->pctl = pinctrl_register(&tb10x_pindesc, &pdev->dev, state);
	if (IS_ERR(state->pctl)) {
		dev_err(&pdev->dev, "could not register TB10x pin driver\n");
		ret = PTR_ERR(state->pctl);
		goto pinctrl_reg_fail;
	}

	return 0;

pinctrl_reg_fail:
	iounmap(state->iobase);
ioremap_fail:
	release_region(mem->start, resource_size(mem));
request_mem_fail:
	mutex_destroy(&state->mutex);
	kfree(state);
	return ret;
}

static int tb10x_pinctrl_remove(struct platform_device *pdev)
{
	struct resource *mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct tb10x_pinctrl_state *state = platform_get_drvdata(pdev);
	int i;

	pinctrl_unregister(state->pctl);

	for (i = 0; i < ARRAY_SIZE(tb10x_pingroups); i++)
		if (tb10x_pingroups[i].pctl == state)
			tb10x_pingroups[i].pctl = NULL;

	iounmap(state->iobase);

	release_region(mem->start, resource_size(mem));

	mutex_destroy(&state->mutex);

	kfree(state);

	return 0;
}

static const struct of_device_id tb10x_pinctrl_dt_ids[] = {
	{ .compatible = TB10X_IOMUX_COMPATIBLE },
	{ }
};
MODULE_DEVICE_TABLE(of, tb10x_pinctrl_dt_ids);

static struct platform_driver tb10x_pinctrl_pdrv = {
	.probe   = tb10x_pinctrl_probe,
	.remove  = tb10x_pinctrl_remove,
	.driver  = {
		.name  = "tb10x_pinctrl",
		.of_match_table = of_match_ptr(tb10x_pinctrl_dt_ids),
		.owner = THIS_MODULE
	}
};

static int __init tb10x_iopinctrl_init(void)
{
	return platform_driver_register(&tb10x_pinctrl_pdrv);
}

static void __exit tb10x_iopinctrl_exit(void)
{
	platform_driver_unregister(&tb10x_pinctrl_pdrv);
}

MODULE_AUTHOR("Christian Ruppert <christian.ruppert@abilis.com>");
MODULE_LICENSE("GPL");
module_init(tb10x_iopinctrl_init);
module_exit(tb10x_iopinctrl_exit);
