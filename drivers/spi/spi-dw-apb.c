/*
 * Memory-mapped APB interface driver for DW SPI Core
 *
 * Code is based on spi_dw_mmio.c
 *
 * Copyright (c) 2013, Synopsys.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>
#include <linux/module.h>

#include "spi-dw.h"

struct dw_apb_spi {
	struct dw_spi	dws;
	struct clk	*clk;
};


static int dw_apb_spi_probe(struct platform_device *pdev)
{
	struct dw_apb_spi *apb_spi;
	struct dw_spi *dws;
	struct resource *res;
	int ret;
	
	apb_spi = devm_kzalloc(&pdev->dev, sizeof(struct dw_apb_spi), GFP_KERNEL);
	if (!apb_spi) {
		return -ENOMEM;
	}

	dws = &apb_spi->dws;

	/* Get basic io resource and map it */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dws->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!dws->regs) {
	    return -ENOMEM;
	}

	dws->irq = platform_get_irq(pdev, 0);
	if (dws->irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		ret = dws->irq; /* -ENXIO */
		return ret;
	}

	apb_spi->clk = devm_clk_get(&pdev->dev, NULL);

	if (IS_ERR(apb_spi->clk))
		return PTR_ERR(apb_spi->clk);
	clk_prepare_enable(apb_spi->clk);
	
	dws->parent_dev = &pdev->dev;
	
	if (pdev->dev.of_node) {
		u32 val = 0;
		/* get the following info from dts ? */
		if (!of_property_read_u32(pdev->dev.of_node,
				"bus-num", &val))
			dws->bus_num = val;
		else
			dws->bus_num  = 0;
		if (!of_property_read_u32(pdev->dev.of_node,
				"num-cs", &val))
			dws->num_cs = val;
		else
			dws->num_cs = 0;

		if (!of_property_read_u32(pdev->dev.of_node,
				"max-freq", &val))
			dws->max_freq = val;
		else
			dws->max_freq = 25000000;
	}

	ret = dw_spi_add_host(dws);
	if (ret) {
		dev_err(&pdev->dev, "add spi host error\n");
		return ret;
	}
	platform_set_drvdata(pdev, apb_spi);
	return 0;
}

static int dw_apb_spi_remove(struct platform_device *pdev)
{
	struct dw_apb_spi *apb_spi = platform_get_drvdata(pdev);

	dw_spi_remove_host(&apb_spi->dws);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id dwapb_of_match_table[] = {
	{ .compatible = "snps, dw-apb-spi"},
	{ /* Sentinel */ }
};
#endif

#ifdef CONFIG_PM
static int dw_apb_spi_suspend(struct device *dev) 
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_apb_spi *apb_spi = platform_get_drvdata(pdev);

	return dw_spi_suspend_host(&apb_spi->dws);

}

static int dw_apb_spi_resume(struct device *dev) 
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dw_apb_spi *apb_spi = platform_get_drvdata(pdev);

	return dw_spi_resume_host(&apb_spi->dws);

}
#endif

static SIMPLE_DEV_PM_OPS(dw_apb_spi_pm_ops, dw_apb_spi_suspend, dw_apb_spi_resume);

static struct platform_driver dw_apb_spi_driver = {
	.probe		= dw_apb_spi_probe,
	.remove		= dw_apb_spi_remove,
	.driver		= {
		.name	= "dw-apb-spi",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dwapb_of_match_table),
		.pm = &dw_apb_spi_pm_ops,
	},
};
module_platform_driver(dw_apb_spi_driver);

MODULE_AUTHOR("Wayne Ren <wei.ren@synopsys.com>");
MODULE_DESCRIPTION("Memory-mapped APB interface driver for DW SPI Core");
MODULE_LICENSE("GPL v2");
