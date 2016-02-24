/*
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include "config_port.h"

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#define DRIVER_NAME		"picoxcell-config-port"

#define CFG_WRITE_PORT		0x0		/* Write port offset */
#define CFG_READ_PORT		0x100		/* Read port offset */
#define DEFAULT_READ_RETRIES	16		/* The number of retries
						 * for an AXI2Cfg config
						 * read */
#define DEFAULT_FIFO_DEPTH	64		/* We have a 64 entry
						 * fifo */

/* Configuration port write bit positions */
#define CAEID_BIT_MASK		(1 << 19)	/* AE ID signal */
#define CADDR_BIT_MASK		(1 << 18)	/* AE ADDR signal */
#define CREAD_BIT_MASK		(1 << 17)	/* READ data signal */
#define CWRITE_BIT_MASK		(1 << 16)	/* WRITE data signal */

#define RB_FAIL_MASK		(1 << 17)	/* Readback failed */
#define RB_VALID_MASK		(1 << 16)	/* Readback valid */

#define GP_AEID			0x0048		/* Device and revision
						 * ID live in array element
						 * 0x48 */
#define DEVICE_ID_OFFSET	0x0030		/* Device ID offset */
#define REVISION_ID_OFFSET	0x0031		/* Revision ID offset */

struct picoxcell_config_port {
	void __iomem *base;
	u32 read_retries;
	u16 fifo_depth;
	spinlock_t lock;
};

int picoxcell_config_port_read(struct device *dev, u16 aeid, u16 ae_addr,
			u16 *buf, u16 count)
{
	u32 val;
	u16 rc, to_read = count;
	unsigned i, retries;
	unsigned long flags;
	struct picoxcell_config_port *cfg = dev_get_drvdata(dev);
	void __iomem *write_p = cfg->base + CFG_WRITE_PORT;
	void __iomem *read_p = cfg->base + CFG_READ_PORT;

	spin_lock_irqsave(&cfg->lock, flags);

	val = aeid | CAEID_BIT_MASK;
	writel(val, write_p);

	while (to_read) {
		/* Output the address to read from */
		val = (ae_addr + (count - to_read)) | CADDR_BIT_MASK;
		writel(val, write_p);

		/* Dispatch the read requests */
		rc = min_t(u16, to_read, cfg->fifo_depth);
		val = CREAD_BIT_MASK | rc;
		writel(val, write_p);

		/* Now read the values */
		for (i = 0; i < rc; ++i) {
			retries = cfg->read_retries;
			while (retries) {
				val = readl(read_p);
				if (val & (RB_VALID_MASK | RB_FAIL_MASK))
					break;
				--retries;
				cpu_relax();
			}

			if (!retries || (val & RB_FAIL_MASK)) {
				pr_warning("config read %04x@%04x failed\n",
					   aeid,
					   (ae_addr + (count - to_read) + i));
				val |= RB_FAIL_MASK;
				break;
			} else
				buf[(count - to_read) + i] = val & 0xFFFF;
		}

		if (val & RB_FAIL_MASK)
			break;

		to_read -= rc;
	}

	spin_unlock_irqrestore(&cfg->lock, flags);

	return !(val & RB_FAIL_MASK) ? count : -EIO;
}
EXPORT_SYMBOL_GPL(picoxcell_config_port_read);

void picoxcell_config_port_write(struct device *dev, u16 aeid, u16 ae_addr,
			  const u16 *buf, u16 count)
{
	u32 val;
	unsigned i;
	unsigned long flags;
	struct picoxcell_config_port *cfg = dev_get_drvdata(dev);
	void __iomem *write_p = cfg->base + CFG_WRITE_PORT;

	spin_lock_irqsave(&cfg->lock, flags);

	/* Output the AEID to read from */
	val = aeid | CAEID_BIT_MASK;
	writel(val, write_p);

	/* Output the address to read from */
	val = ae_addr | CADDR_BIT_MASK;
	writel(val, write_p);

	for (i = 0; i < count; ++i) {
		val = buf[i] | CWRITE_BIT_MASK;
		writel(val, write_p);
	}

	spin_unlock_irqrestore(&cfg->lock, flags);
}
EXPORT_SYMBOL_GPL(picoxcell_config_port_write);

void picoxcell_config_port_write_buf(struct device *dev, const u32 *buf, unsigned nr_words)
{
	unsigned i;
	unsigned long flags;
	struct picoxcell_config_port *cfg = dev_get_drvdata(dev);
	void __iomem *write_p = cfg->base + CFG_WRITE_PORT;

	spin_lock_irqsave(&cfg->lock, flags);

	for (i = 0; i < nr_words; ++i)
		writel(*buf++, write_p);

	spin_unlock_irqrestore(&cfg->lock, flags);
}
EXPORT_SYMBOL_GPL(picoxcell_config_port_write_buf);

static int picoxcell_config_port_probe(struct platform_device *pdev)
{
	struct picoxcell_config_port *priv;
	struct resource *res;
	u16 devid;
	u16 revid;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->read_retries = DEFAULT_READ_RETRIES;
	if (pdev->dev.of_node) {
		of_property_read_u32(pdev->dev.of_node,
				     "picochip,read-retries",
				     &priv->read_retries);
	}

	priv->fifo_depth = DEFAULT_FIFO_DEPTH;
	if (pdev->dev.of_node) {
		of_property_read_u16(pdev->dev.of_node,
				     "picochip,fifo-depth",
				     &priv->fifo_depth);
	}

	spin_lock_init(&priv->lock);

	platform_set_drvdata(pdev, priv);

	ret = picoxcell_config_port_read(&pdev->dev, GP_AEID, DEVICE_ID_OFFSET, &devid, 1);
	if (ret == 1) {
		ret = picoxcell_config_port_read(&pdev->dev, GP_AEID, REVISION_ID_OFFSET, &revid, 1);
		if (ret == 1) {
			if (devid != 0x00000021 && devid != 0x00000022) {
				dev_err(&pdev->dev, "Attached picoArray (0x%04x) "
						    "is not a pc3x3 device\n",
					devid);
				return -ENODEV;
			}

			dev_info(&pdev->dev, "Attached a pc3%d3 (revision 0x%04x) "
					     "device.  Read retries %u, "
					     "FIFO depth %u.\n",
				 (devid - 31), revid, priv->read_retries, priv->fifo_depth);
		} else {
			dev_warn(&pdev->dev, "picoArray is not yet up\n");
		}
	} else {
		dev_warn(&pdev->dev, "picoArray is not yet up\n");
	}

	return 0;
}

static int picoxcell_config_port_remove(struct platform_device *pdev)
{
	return 0; /* nothing to do */
}

static const struct of_device_id picoxcell_config_port_of_match[] = {
	{ .compatible = "picochip,pc3x3-config-port", },
	{ }
};
MODULE_DEVICE_TABLE(of, picoxcell_config_port_of_match);

static struct platform_driver picoxcell_config_port_driver = {
	.probe		= picoxcell_config_port_probe,
	.remove		= picoxcell_config_port_remove,
	.driver		= {
		.name		= DRIVER_NAME,
		.of_match_table	= picoxcell_config_port_of_match,
	},
};
module_platform_driver(picoxcell_config_port_driver);

MODULE_AUTHOR("Jamie Iles <jamie@jamieiles.com>, "
	      "Michael van der Westhuizen <michael@smart-africa.com>");
MODULE_DESCRIPTION("");
MODULE_LICENSE("GPL v2");
