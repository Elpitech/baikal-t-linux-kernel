// SPDX-License-Identifier: GPL-2.0
/*
 * Linux 4.9.x <- 5.1.x compatibility header file
 *
 * Copyright (C) 2018-2019 T-platforms JSC (fancer.lancer@gmail.com)
 */

#ifndef KERNEL_COMPAT_H
#define KERNEL_COMPAT_H

#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/slab.h>

/* This method has been exported only since kernel 4.20 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,20,0)
#if IS_ENABLED(CONFIG_OF)
static inline int __spi_of_device_match(struct device *dev, void *data)
{
	return dev->of_node == data;
}

/* must call put_device() when done with returned spi_device device */
static inline struct spi_device *of_find_spi_device_by_node(struct device_node *node)
{
	struct device *dev = bus_find_device(&spi_bus_type, NULL, node,
						__spi_of_device_match);
	return dev ? to_spi_device(dev) : NULL;
}
#endif /* IS_ENABLED(CONFIG_OF) */
#endif

/* Callback wrapper has been available since kernel 4.11.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
static inline struct dma_async_tx_descriptor *dmaengine_prep_dma_memcpy(
		struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		size_t len, unsigned long flags)
{
	if (!chan || !chan->device || !chan->device->device_prep_dma_memcpy)
		return NULL;

	return chan->device->device_prep_dma_memcpy(chan, dest, src,
						    len, flags);
}
#endif

/* SPI-master naming has been changed to SPI-controller since 4.13.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static inline void *kmalloc_array_node(size_t n, size_t size, gfp_t flags,
				       int node)
{
	if (size != 0 && n > SIZE_MAX / size)
		return NULL;
	if (__builtin_constant_p(n) && __builtin_constant_p(size))
		return kmalloc_node(n * size, flags, node);
	return __kmalloc_node(n * size, flags, node);
}

static inline void *kcalloc_node(size_t n, size_t size, gfp_t flags, int node)
{
	return kmalloc_array_node(n, size, flags | __GFP_ZERO, node);
}
#endif

/*
 * Some headers and declrations have been moved since kernel 4.11.0
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
#include <linux/sched/types.h>
#include <linux/sched/signal.h>
#else
#include <linux/sched.h>
#include <linux/signal.h>
#endif

/* SPI-master naming has been changed to SPI-controller since 4.13.0 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,13,0)
#define spi_controller				spi_master

#define SPI_CONTROLLER_HALF_DUPLEX		SPI_MASTER_HALF_DUPLEX
#define SPI_CONTROLLER_NO_RX			SPI_MASTER_NO_RX
#define SPI_CONTROLLER_NO_TX			SPI_MASTER_NO_TX
#define SPI_CONTROLLER_MUST_RX			SPI_MASTER_MUST_RX
#define SPI_CONTROLLER_MUST_TX			SPI_MASTER_MUST_TX

#define spi_controller_get_devdata(_ctlr)	spi_master_get_devdata(_ctlr)
#define spi_controller_set_devdata(_ctlr, _data) \
	spi_master_set_devdata(_ctlr, _data)
#define spi_controller_get(_ctlr)		spi_master_get(_ctlr)
#define spi_controller_put(_ctlr)		spi_master_put(_ctlr)
#define spi_controller_suspend(_ctlr)		spi_master_suspend(_ctlr)
#define spi_controller_resume(_ctlr)		spi_master_resume(_ctlr)

#define spi_register_controller(_ctlr)		spi_register_master(_ctlr)
#define devm_spi_register_controller(_dev, _ctlr) \
	devm_spi_register_master(_dev, _ctlr)
#define spi_unregister_controller(_ctlr)	spi_unregister_master(_ctlr)
#endif

#endif /* KERNEL_COMPAT_H */
