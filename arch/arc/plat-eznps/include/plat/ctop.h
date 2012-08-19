/*******************************************************************************

  Copyright(c) 2012 EZchip Technologies.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

*******************************************************************************/

#ifndef _PLAT_EZNPS_CTOP_H
#define _PLAT_EZNPS_CTOP_H


/* core auxiliary registers */
#define CTOP_AUX_BASE		0xFFFFF800
#define CTOP_AUX_IACK		(CTOP_AUX_BASE + 0x088)
#define CTOP_AUX_UDMC		(CTOP_AUX_BASE + 0x300)

#define CTOP_UDMC_NAT_OFFSET	0x8
#define CTOP_UDMC_NAT_MASK	(0x7 << CTOP_UDMC_NAT_OFFSET)


#endif /* _PLAT_EZNPS_CTOP_H */
