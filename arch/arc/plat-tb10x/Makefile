# Abilis Systems TB10x platform Makefile
#
# Author: Christian Ruppert <christian.ruppert@abilis.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA


KBUILD_CFLAGS   += -Iarch/arc/plat-tb10x/include
obj-$(CONFIG_PINCTRL_TB10X)	+= ../../../drivers/pinctrl/pinctrl-tb10x.o
obj-$(CONFIG_GPIO_TB10X)	+= ../../../drivers/gpio/gpio-tb10x.o
obj-$(CONFIG_SPI_TB100)		+= ../../../drivers/spi/spi-tb100.o

obj-y += tb10x.o
