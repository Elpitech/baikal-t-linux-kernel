/*
 * Baikal-T SOC platform support code. Boot Controller driver.
 *
 * Copyright (C) 2017 T-platforms JSC
 *
 * Author:
 *   Sergey Semin <Sergey.Semin@t-platforms.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <linux/property.h>


#define DRIVER_NAME 		"be-bc"
#define VERSION			"1.03"

#define BE_BC_CSR		0x00
#define BE_BC_MAR		0x04
#define BE_BC_DRID		0x08
#define BE_BC_VID		0x0C
#define BE_BC_CSR_BMODE		(0x3 << 0)
#define BE_BC_CSR_SPI_RDA	(0x1 << 8)
#define BE_BC_CSR_SPI_MDELAY	1
#define BE_BC_OFF		0   /* transparent mode of spi memory */
#define BE_BC_ON		1   /* not transparent */
#define BE_BC_RESET		1

struct be_bc {
	void __iomem *regs;
};

static void bc_enable_spi(struct be_bc *bc)
{
	writel(BE_BC_CSR_SPI_RDA, bc->regs + BE_BC_CSR);
	msleep(BE_BC_CSR_SPI_MDELAY);
}

static void bc_disable_spi(struct be_bc *bc)
{
	writel(readl(bc->regs + BE_BC_CSR) & ~BE_BC_CSR_SPI_RDA,
	       bc->regs + BE_BC_CSR);
	msleep(BE_BC_CSR_SPI_MDELAY);
}

static int be_bc_drv_probe(struct platform_device *pdev)
{
	struct be_bc *bc;
	struct resource *res;
	u32 vid, drid;
	int rc;

	bc = devm_kzalloc(&pdev->dev, sizeof(*bc), GFP_KERNEL);
	if (!bc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bc->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(bc->regs))
		return PTR_ERR(bc->regs);

	platform_set_drvdata(pdev, bc);
	bc_enable_spi(bc);

	drid = readl(bc->regs + BE_BC_DRID);
	vid  = readl(bc->regs + BE_BC_VID);

	dev_info(&pdev->dev, "Baikal Electronics Boot Controller Driver\n");
	dev_info(&pdev->dev, "VID: 0x%08x, DRID: 0x%08x\n", vid, drid);
	dev_info(&pdev->dev, "Version " VERSION "\n");

	rc = of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);

	return rc;
}

static int be_bc_drv_remove(struct platform_device *pdev)
{
	struct be_bc *bc;

	dev_info(&pdev->dev, "removing %s child devices...\n", DRIVER_NAME);
	of_platform_depopulate(&pdev->dev);
	bc = platform_get_drvdata(pdev);
	bc_disable_spi(bc);

	return 0;
}

static const struct of_device_id be_bc_of_match[] = {
	{ .compatible = "be,bc", },
	{ /* end of table */}
};

MODULE_DEVICE_TABLE(of, be_bc_of_match);

static struct platform_driver be_bc_driver = {
	.probe		= be_bc_drv_probe,
	.remove		= be_bc_drv_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(be_bc_of_match),
	},
};

module_platform_driver(be_bc_driver);

MODULE_VERSION(VERSION);
MODULE_AUTHOR("Sergey Semin <Sergey.Semin@t-platforms.ru>");
MODULE_DESCRIPTION("Baikal Electronics Boot Controller Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:be_bc");
