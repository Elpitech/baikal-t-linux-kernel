obj-$(CONFIG_FB_SM750)	+= sm750fb.o

sm750fb-objs		:= sm750.o sm750_hw.o sm750_accel.o sm750_cursor.o ddk750_chip.o ddk750_power.o ddk750_mode.o
sm750fb-objs		+= ddk750_display.o ddk750_help.o ddk750_swi2c.o ddk750_sii164.o ddk750_dvi.o ddk750_hwi2c.o
sm750fb-objs		+= sm750_ddc.o
ifeq ($(CONFIG_SM750_DMA),y)
sm750fb-objs		+= dw_pci_dma.o
endif
ccflags-y		+= -DSII164_FULL_FUNCTIONS
