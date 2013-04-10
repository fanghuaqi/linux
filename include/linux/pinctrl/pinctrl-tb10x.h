/*
 * Abilis Systems TB10x pin control driver
 *
 * Copyright (C) Abilis Systems 2013
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

#ifndef PINCTRL_TB10X_H
#define PINCTRL_TB10X_H

struct tb10x_pinfuncgrp; /* Opaque structure */

#ifdef CONFIG_PINCTRL_TB10X
struct tb10x_pinfuncgrp *tb10x_prepare_gpio_range(struct device_node *dn,
					struct pinctrl_gpio_range *gr);

void tb10x_setup_gpio_range(struct tb10x_pinfuncgrp *pfg,
					struct pinctrl_gpio_range *gr);
#else
/*static inline struct tb10x_pinfuncgrp *tb10x_prepare_gpio_range(
			struct device_node *dn, struct pinctrl_gpio_range *gr)
{
	return -ENXIO;
}

static inline void tb10x_setup_gpio_range(struct tb10x_pinfuncgrp *pfg,
					struct pinctrl_gpio_range *gr)
{
}*/
#endif

#endif
