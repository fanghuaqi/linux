#
# Makefile for the linux kernel.
#

KBUILD_CFLAGS += -Iarch/arc/plat-eznps/include

obj-y := platform.o irq.o
obj-$(CONFIG_SMP) += time.o smp.o head.o
