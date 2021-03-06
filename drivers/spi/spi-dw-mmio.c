/*
 * Memory-mapped interface driver for DW SPI Core
 *
 * Copyright (c) 2010, Octasic semiconductor.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#ifdef CONFIG_MIPS_BAIKAL
#include <asm/mach-baikal/bc.h>
#endif

#include "spi-dw.h"

#define DRIVER_NAME "dw_spi_mmio"

#ifdef CONFIG_SPI_DW_MMIO_DMA
extern void dw_spi_mmio_dma_init(struct dw_spi *dws);
#else
inline void dw_spi_mmio_dma_init(struct dw_spi *dws) {}
#endif

struct dw_spi_mmio {
	struct dw_spi  dws;
#ifdef CONFIG_MIPS_BAIKAL
	struct be_bc   *bc;
#endif
	struct clk     *clk;
};

#ifdef CONFIG_MIPS_BAIKAL
static int dw_spi_mmio_boot_enable(struct dw_spi_mmio *dwsmmio,
				    struct platform_device *pdev)
{
	struct device_node *bc_np;

	/* Enable the DW SPI controller interface if presented */
	bc_np = of_parse_phandle(pdev->dev.of_node, "boot-controller", 0);
	if (bc_np) {
		dwsmmio->bc = of_find_be_bc_device_by_node(bc_np);
		of_node_put(bc_np);
		if (!dwsmmio->bc) {
			dev_err(&pdev->dev, "missing boot-controller of-node\n");
			return -EINVAL;
		}
		be_bc_enable_spi(dwsmmio->bc);
	}

	return 0;
}

static void dw_spi_mmio_boot_disable(struct dw_spi_mmio *dwsmmio)
{
	if (dwsmmio->bc)
		be_bc_disable_spi(dwsmmio->bc);
}
#else
static int dw_spi_mmio_boot_enable(struct dw_spi_mmio *dwsmmio,
				   struct platform_device *pdev) {
	return 0;
}

static void dw_spi_mmio_boot_disable(struct dw_spi_mmio *dwsmmio) {}
#endif

static int dw_spi_mmio_probe(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio;
	struct dw_spi *dws;
	struct resource *mem;
	int ret;
	int num_cs;

	dwsmmio = devm_kzalloc(&pdev->dev, sizeof(struct dw_spi_mmio),
			GFP_KERNEL);
	if (!dwsmmio)
		return -ENOMEM;

	dws = &dwsmmio->dws;

	/* Get basic io resource and map it */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(dws->regs)) {
		dev_err(&pdev->dev, "SPI region map failed\n");
		return PTR_ERR(dws->regs);
	}
	dws->paddr = mem->start;

	/* IRQ might be unavailable */
	dws->irq = platform_get_irq(pdev, 0);

	dwsmmio->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(dwsmmio->clk))
		return PTR_ERR(dwsmmio->clk);
	ret = clk_prepare_enable(dwsmmio->clk);
	if (ret)
		return ret;

	dws->bus_num = of_alias_get_id(pdev->dev.of_node, "ssi");

	dws->max_freq = clk_get_rate(dwsmmio->clk);

	device_property_read_u32(&pdev->dev, "reg-io-width", &dws->reg_io_width);

	num_cs = 4;

	device_property_read_u32(&pdev->dev, "num-cs", &num_cs);

	dws->num_cs = num_cs;

	if (pdev->dev.of_node) {
		int i;

		for (i = 0; i < dws->num_cs; i++) {
			int cs_gpio = of_get_named_gpio(pdev->dev.of_node,
					"cs-gpios", i);

			if (cs_gpio == -EPROBE_DEFER) {
				ret = cs_gpio;
				goto out;
			}

			if (gpio_is_valid(cs_gpio)) {
				ret = devm_gpio_request(&pdev->dev, cs_gpio,
						dev_name(&pdev->dev));
				if (ret)
					goto out;
			}
		}
	}

	ret = dw_spi_mmio_boot_enable(dwsmmio, pdev);
	if (ret)
		goto out;

	dw_spi_mmio_dma_init(dws);

	pdev->dev.dma_mask = NULL;
	ret = dw_spi_add_host(&pdev->dev, dws);
	if (ret)
		goto out_boot;

	/* Dump DW component type */
	ret = readl(dws->regs + DW_SPI_VERSION);
	dev_info(&pdev->dev, "DW SPI ID: 0x%08X, Version: %c.%c%c%c\n",
		readl(dws->regs + DW_SPI_IDR), (ret >> 24) & 0xff,
		(ret >> 16) & 0xff, (ret >> 8) & 0xff, ret & 0xff);

	platform_set_drvdata(pdev, dwsmmio);
	return 0;

out_boot:
	dw_spi_mmio_boot_disable(dwsmmio);

out:
	clk_disable_unprepare(dwsmmio->clk);
	return ret;
}

static int dw_spi_mmio_remove(struct platform_device *pdev)
{
	struct dw_spi_mmio *dwsmmio = platform_get_drvdata(pdev);

	dw_spi_remove_host(&dwsmmio->dws);

	dw_spi_mmio_boot_disable(dwsmmio);

	clk_disable_unprepare(dwsmmio->clk);

	return 0;
}

static const struct of_device_id dw_spi_mmio_of_match[] = {
	{ .compatible = "snps,dw-apb-ssi", },
	{ /* end of table */}
};
MODULE_DEVICE_TABLE(of, dw_spi_mmio_of_match);

static struct platform_driver dw_spi_mmio_driver = {
	.probe		= dw_spi_mmio_probe,
	.remove		= dw_spi_mmio_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = dw_spi_mmio_of_match,
	},
};
module_platform_driver(dw_spi_mmio_driver);

MODULE_AUTHOR("Jean-Hugues Deschenes <jean-hugues.deschenes@octasic.com>");
MODULE_DESCRIPTION("Memory-mapped I/O interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
