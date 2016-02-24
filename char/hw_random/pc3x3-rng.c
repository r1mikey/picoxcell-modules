/*
 * Intel/Picochip picoXcell pc3x3 SoC Random Number Generarator Driver
 *
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 *
 * Derived in part from drivers/char/hw_random/picoxcell-rng.c
 *   Copyright (c) 2010-2011 Picochip Ltd., Jamie Iles
 *   Author: Jamie Iles <jamie@jamieiles.com>
 *
 * Derived in part from drivers/char/hw_random/timeriomem-rng.c
 *   Copyright (C) 2009 Alexander Clouter <alex@digriz.org.uk>
 *   Author: Alexander Clouter <alex@digriz.org.uk>
 *
 * Derived in part from drivers/char/hw_random/omap-rng.c
 *   Copyright 2005 (c) MontaVista Software, Inc.
 *   Author: Deepak Saxena <dsaxena@plexity.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#define DATA_REG_OFFSET		0x0200
#define CSR_REG_OFFSET		0x0278
#define CSR_OUT_EMPTY_MASK	(1 << 24)
#define CSR_OUT_HALF_MASK	(1 << 25)
#define CSR_OUT_FULL_MASK	(1 << 26)
#define CSR_FAULT_MASK		(1 << 1)
#define TRNG_BLOCK_RESET_MASK	(1 << 0)
#define TAI_REG_OFFSET		0x0380

/*
 * The maximum amount of time in microseconds to spend waiting for data if the
 * core wants us to wait.  The TRNG should generate 32 bits every 320ns so a
 * timeout of 20us seems reasonable.  The TRNG does builtin tests of the data
 * for randomness so we can't always assume there is data present.
 */
#define PC3X3_DEFAULT_RNG_WAIT_TIMEOUT	20
#define PC3X3_DEFAULT_RNG_QUALITY	768
#define PC3X3_MAX_RNG_QUALITY		1024

#define PC3X3_OUT_EMPTY_BYTES		0
#define PC3X3_OUT_NOT_EMPTY_BYTES	4
#define PC3X3_OUT_HALF_BYTES		16
#define PC3X3_OUT_FULL_BYTES		32

struct pc3x3_rng_data {
	void __iomem		*io_base;
	u32			timeout;
	struct clk		*rng_clk;
	struct hwrng		ops;
	struct platform_device	*pdev;
};

#define to_rng_priv(rng) ((struct pc3x3_rng_data *)rng->priv)

static inline u32 pc3x3_rng_read_csr(struct pc3x3_rng_data *priv)
{
	return __raw_readl(priv->io_base + CSR_REG_OFFSET);
}

static inline int pc3x3_trng_is_empty(struct pc3x3_rng_data *priv)
{
	return pc3x3_rng_read_csr(priv) & CSR_OUT_EMPTY_MASK;
}

static inline int pc3x3_rng_has_fault(struct pc3x3_rng_data *priv)
{
	return pc3x3_rng_read_csr(priv) & CSR_FAULT_MASK;
}

/*
 * Take the random number generator out of reset and make sure the interrupts
 * are masked. We shouldn't need to get large amounts of random bytes so just
 * poll the status register. The hardware generates 32 bits every 320ns so we
 * shouldn't have to wait long enough to warrant waiting for an IRQ.
 */
static void pc3x3_rng_start(struct pc3x3_rng_data *priv)
{
	__raw_writel(0, priv->io_base + TAI_REG_OFFSET);
	__raw_writel(0, priv->io_base + CSR_REG_OFFSET);
}

static void pc3x3_rng_stop(struct pc3x3_rng_data *priv)
{
	__raw_writel(TRNG_BLOCK_RESET_MASK, priv->io_base + CSR_REG_OFFSET);
	__raw_writel(TRNG_BLOCK_RESET_MASK, priv->io_base + TAI_REG_OFFSET);
}

static void pc3x3_rng_reset(struct pc3x3_rng_data *priv)
{
	pc3x3_rng_stop(priv);
	pc3x3_rng_start(priv);
}

static void pc3x3_rng_clear_fault(struct pc3x3_rng_data *priv)
{
	dev_err_ratelimited(&priv->pdev->dev,
			    "fault detected, resetting RNG\n");
	pc3x3_rng_reset(priv);
}

static size_t pc3x3_rng_bytes_avail(struct pc3x3_rng_data *priv)
{
	u32 csr = pc3x3_rng_read_csr(priv);

	if (csr & CSR_OUT_FULL_MASK)
		return PC3X3_OUT_FULL_BYTES;
	else if (csr & CSR_OUT_HALF_MASK)
		return PC3X3_OUT_HALF_BYTES;
	else if (csr & CSR_OUT_EMPTY_MASK)
		return PC3X3_OUT_EMPTY_BYTES;

	return PC3X3_OUT_NOT_EMPTY_BYTES;
}

static void pc3x3_wait_for_bytes(struct pc3x3_rng_data *priv, size_t max)
{
	u32 i;
	size_t await = PC3X3_OUT_NOT_EMPTY_BYTES;

	if (max > PC3X3_OUT_NOT_EMPTY_BYTES &&
	    max <= PC3X3_OUT_HALF_BYTES)
		await = PC3X3_OUT_HALF_BYTES;
	else
		await = PC3X3_OUT_FULL_BYTES;

	if (pc3x3_rng_bytes_avail(priv) >= await)
		return;

	for (i = 0; i < priv->timeout; ++i) {
		udelay(1);

		if (pc3x3_rng_bytes_avail(priv) >= await)
			return;

		if (pc3x3_rng_has_fault(priv))
			pc3x3_rng_clear_fault(priv);
	}
}

static int pc3x3_rng_read(struct hwrng *rng, void *data, size_t max, bool wait)
{
	struct pc3x3_rng_data *priv = to_rng_priv(rng);
	int ret = 0;

	if (max < PC3X3_OUT_NOT_EMPTY_BYTES)
		return ret;

	max = round_down(max, PC3X3_OUT_NOT_EMPTY_BYTES);

	if (pc3x3_rng_bytes_avail(priv) < max)
		pc3x3_wait_for_bytes(priv, max);

	while (max && !pc3x3_trng_is_empty(priv) &&
	       !pc3x3_rng_has_fault(priv)) {
		*(u32 *)data = __raw_readl(priv->io_base + DATA_REG_OFFSET);
		data += PC3X3_OUT_NOT_EMPTY_BYTES;
		max -= PC3X3_OUT_NOT_EMPTY_BYTES;
		ret += PC3X3_OUT_NOT_EMPTY_BYTES;
	}

	if (pc3x3_rng_has_fault(priv))
		pc3x3_rng_clear_fault(priv);

	return ret;
}

static int pc3x3_rng_probe(struct platform_device *pdev)
{
	struct pc3x3_rng_data *priv;
	struct resource *res;
	u32 quality;
	int ret = 0;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "pc3x3_rng OF data is missing\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	if (res->start % 4 != 0 || resource_size(res) != 4) {
		dev_err(&pdev->dev,
			"address must be 32 bits wide and 32 bit aligned\n");
		return -EINVAL;
	}

	/* Allocate memory for the device structure (and zero it) */
	priv = devm_kzalloc(&pdev->dev,
			    sizeof(struct pc3x3_rng_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->timeout = PC3X3_DEFAULT_RNG_WAIT_TIMEOUT;
	of_property_read_u32(pdev->dev.of_node,
			     "wait-timeout-us", &priv->timeout);

	quality = PC3X3_DEFAULT_RNG_QUALITY;
	of_property_read_u32(pdev->dev.of_node,
			     "rng-quality-ppm", &quality);
	quality = min_t(u32, PC3X3_MAX_RNG_QUALITY, quality);

	priv->pdev = pdev;
	platform_set_drvdata(pdev, priv);

	priv->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->io_base))
		return PTR_ERR(priv->io_base);

	priv->rng_clk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(priv->rng_clk))
		return PTR_ERR(priv->rng_clk);

	ret = clk_prepare_enable(priv->rng_clk);
	if (ret)
		return ret;

	pc3x3_rng_stop(priv);
	pc3x3_rng_start(priv);

	priv->ops.name		= dev_name(&pdev->dev);
	priv->ops.read		= pc3x3_rng_read;
	priv->ops.priv		= (unsigned long)priv;
	priv->ops.quality	= (unsigned short)quality;

	ret = hwrng_register(&priv->ops);
	if (ret) {
		pc3x3_rng_stop(priv);
		clk_disable_unprepare(priv->rng_clk);
		return ret;
	}

	dev_dbg(&pdev->dev,
		"picoXcell pc3x3 random number generator active\n");
	return 0;
}

static int pc3x3_rng_remove(struct platform_device *pdev)
{
	struct pc3x3_rng_data *priv = platform_get_drvdata(pdev);

	hwrng_unregister(&priv->ops);
	pc3x3_rng_stop(priv);
	clk_disable_unprepare(priv->rng_clk);

	return 0;
}

#ifdef CONFIG_PM
static int pc3x3_rng_suspend(struct device *dev)
{
	struct pc3x3_rng_data *priv = dev_get_drvdata(dev);

	clk_disable(priv->rng_clk);

	return 0;
}

static int pc3x3_rng_resume(struct device *dev)
{
	struct pc3x3_rng_data *priv = dev_get_drvdata(dev);

	return clk_enable(priv->rng_clk);
}

static const struct dev_pm_ops pc3x3_rng_pm_ops = {
	.suspend	= pc3x3_rng_suspend,
	.resume		= pc3x3_rng_resume,
};
#endif /* CONFIG_PM */

static const struct of_device_id pc3x3_rng_match[] = {
	{ .compatible = "picochip,pc3x3-rng" },
	{},
};
MODULE_DEVICE_TABLE(of, pc3x3_rng_match);

static struct platform_driver pc3x3_rng_driver = {
	.driver = {
		.name		= "pc3x3-rng",
		.owner		= THIS_MODULE,
		.of_match_table	= pc3x3_rng_match,
#ifdef CONFIG_PM
		.pm		= &pc3x3_rng_pm_ops,
#endif /* CONFIG_PM */
	},
	.probe		= pc3x3_rng_probe,
	.remove		= pc3x3_rng_remove,
};

module_platform_driver(pc3x3_rng_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michael van der Westhuizen <michael@smart-africa.com>");
MODULE_DESCRIPTION("Intel/Picochip picoXcell pc3x3 RNG driver");
