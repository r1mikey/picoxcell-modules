/*
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 * Copyright (c) 2011 Picochip Ltd., Jamie Iles
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Based on arch/arm/mach-picoxcell/bus_err.c from the picoXcell BSP along
 * with data from the pc30xx.c, pc3x2.c and pc3x3.c files in that directory.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/edac.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>

/* ugh... */
#include <../drivers/edac/edac_core.h>
#include <../drivers/edac/edac_module.h>

#define COMPAT_PC30XX		"picochip,pc30xx-bus-edac"
#define COMPAT_PC3X2		"picochip,pc3x2-bus-edac"
#define COMPAT_PC3X3		"picochip,pc3x3-bus-edac"

#define IRQ_ENABLE_OFFSET	0x00
#define IRQ_CLEAR_OFFSET	0x04
#define IRQ_MASK_OFFSET		0x08
#define IRQ_TEST_OFFSET		0x0c
#define IRQ_PRE_MASK_OFFSET	0x10
#define IRQ_POST_MASK_OFFSET	0x14

#define IRQ_MASK_NONE		0
#define IRQ_ENABLE_ALL		0x00ffffff
#define IRQ_ENABLE_ALL_PC3X3	0x00bffbff

#define IRQ_CLEAR_ALL		IRQ_ENABLE_ALL
#define IRQ_CLEAR_ALL_PC3X3	IRQ_ENABLE_ALL_PC3X3

#define MAX_SNOOP_NAMES	32

static const char *pc30xx_snoop_names[MAX_SNOOP_NAMES] = {
	[0]	= "dmac1_channel0 (read)",
	[1]	= "dmac1_channel1 (read)",
	[2]	= "dmac1_channel2 (read)",
	[3]	= "dmac1_channel3 (read)",
	[4]	= "dmac2_channel0 (read)",
	[5]	= "dmac2_channel1 (read)",
	[6]	= "dmac2_channel2 (read)",
	[7]	= "dmac2_channel3 (read)",
	[8]	= "emac (read)",
	[9]	= "cipher (read)",
	[10]	= "nand (read)",
	[11]	= "ipsec (read)",
	[12]	= "dmac1_channel0 (write)",
	[13]	= "dmac1_channel1 (write)",
	[14]	= "dmac1_channel2 (write)",
	[15]	= "dmac1_channel3 (write)",
	[16]	= "dmac2_channel0 (write)",
	[17]	= "dmac2_channel1 (write)",
	[18]	= "dmac2_channel2 (write)",
	[19]	= "dmac2_channel3 (write)",
	[20]	= "emac (write)",
	[21]	= "cipher (write)",
	[22]	= "nand (write)",
	[23]	= "ipsec (write)",
};

static const char *pc3x2_snoop_names[MAX_SNOOP_NAMES] = {
	[0]	= "dmac1_channel0 (read)",
	[1]	= "dmac1_channel1 (read)",
	[2]	= "dmac1_channel2 (read)",
	[3]	= "dmac1_channel3 (read)",
	[4]	= "dmac2_channel0 (read)",
	[5]	= "dmac2_channel1 (read)",
	[6]	= "dmac2_channel2 (read)",
	[7]	= "dmac2_channel3 (read)",
	[8]	= "emac (read)",
	[9]	= "cipher (read)",
	[10]	= "srtp (read)",
	[11]	= "ipsec (read)",
	[12]	= "dmac1_channel0 (write)",
	[13]	= "dmac1_channel1 (write)",
	[14]	= "dmac1_channel2 (write)",
	[15]	= "dmac1_channel3 (write)",
	[16]	= "dmac2_channel0 (write)",
	[17]	= "dmac2_channel1 (write)",
	[18]	= "dmac2_channel2 (write)",
	[19]	= "dmac2_channel3 (write)",
	[20]	= "emac (write)",
	[21]	= "cipher (write)",
	[22]	= "srtp (write)",
	[23]	= "ipsec (write)",
};

static const char *pc3x3_snoop_names[MAX_SNOOP_NAMES] = {
	[0]	= "dmac1_channel0 (read)",
	[1]	= "dmac1_channel1 (read)",
	[2]	= "dmac1_channel2 (read)",
	[3]	= "dmac1_channel3 (read)",
	[4]	= "dmac2_channel0 (read)",
	[5]	= "dmac2_channel1 (read)",
	[6]	= "dmac2_channel2 (read)",
	[7]	= "dmac2_channel3 (read)",
	[8]	= "emac (read)",
	[9]	= "cipher (read)",
	[11]	= "ipsec (read)",
	[12]	= "dmac1_channel0 (write)",
	[13]	= "dmac1_channel1 (write)",
	[14]	= "dmac1_channel2 (write)",
	[15]	= "dmac1_channel3 (write)",
	[16]	= "dmac2_channel0 (write)",
	[17]	= "dmac2_channel1 (write)",
	[18]	= "dmac2_channel2 (write)",
	[19]	= "dmac2_channel3 (write)",
	[20]	= "emac (write)",
	[21]	= "cipher (write)",
	[23]	= "ipsec (write)",
};

static const struct of_device_id picoxcell_bus_err_of_match[] = {
	{ .compatible = COMPAT_PC30XX, .data = pc30xx_snoop_names, },
	{ .compatible = COMPAT_PC3X2, .data = pc3x2_snoop_names, },
	{ .compatible = COMPAT_PC3X3, .data = pc3x3_snoop_names, },
	{},
};

struct picoxcell_snoop_drvdata {
	void __iomem *base;
	const char **names;
	unsigned int test_mask;
};

static const char *picoxcell_err_name(
	struct picoxcell_snoop_drvdata *drvdata, int bit)
{
	return drvdata->names[bit] ?: "<INVALID SNOOP ERROR>";
}

static irqreturn_t picoxcell_bus_err_handler(int irq, void *dev_id)
{
	struct edac_device_ctl_info *dci = dev_id;
	struct picoxcell_snoop_drvdata *drvdata = dci->pvt_info;
	unsigned long axi_error =
		readl(drvdata->base + IRQ_POST_MASK_OFFSET);
	int bit;

	for_each_set_bit(bit, &axi_error, 32) {
		edac_device_handle_ue(dci, 0, 0,
				      picoxcell_err_name(drvdata, bit));
		writel(1U << bit, drvdata->base + IRQ_CLEAR_OFFSET);
	}

	return IRQ_HANDLED;
}

static ssize_t inject_error(struct edac_device_ctl_info *dci,
			    const char *buffer, size_t count)
{
	struct picoxcell_snoop_drvdata *drvdata = dci->pvt_info;
	int res;
	unsigned int value;

	res = kstrtouint(buffer, 0, &value);
	if (res < 0)
		return (ssize_t)res;
	value &= drvdata->test_mask;

	if (value) {
		dev_info(dci->dev, "Injecting fault mask 0x%08x\n", value);
		writel(value, drvdata->base + IRQ_TEST_OFFSET);
	}

	return count;
}

static struct edac_dev_sysfs_attribute picoxcell_bus_err_sysfs_attributes[] = {
	{
		.attr	= {
			.name = "inject_error",
			.mode = (S_IRUGO|S_IWUSR)
		},
		.show	= NULL,
		.store	= inject_error
	},
	{ .attr = { .name = NULL } }
};

static int picoxcell_bus_err_probe(struct platform_device *pdev)
{
	const struct of_device_id *id;
	struct edac_device_ctl_info *dci;
	struct picoxcell_snoop_drvdata *drvdata;
	struct resource *r;
	int res = 0;

	if (!pdev->dev.of_node)
		return -EFAULT;

	dci = edac_device_alloc_ctl_info(sizeof(*drvdata), "bus",
		1, "axi", 1, 2, NULL, 0, 0);
	if (!dci)
		return -ENOMEM;

	if (of_property_read_bool(pdev->dev.of_node, "default-panic-on-ue"))
		dci->panic_on_ue = 1;
	drvdata = dci->pvt_info;
	dci->dev = &pdev->dev;
	platform_set_drvdata(pdev, dci);

	if (!devres_open_group(&pdev->dev, NULL, GFP_KERNEL))
		return -ENOMEM;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		res = -ENODEV;
		goto err;
	}

	if (!devm_request_mem_region(&pdev->dev, r->start,
				     resource_size(r), dev_name(&pdev->dev))) {
		dev_err(&pdev->dev, "Error while requesting mem region\n");
		res = -EBUSY;
		goto err;
	}

	drvdata->base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (!drvdata->base) {
		dev_err(&pdev->dev, "Unable to map regs\n");
		res = -ENOMEM;
		goto err;
	}

	id = of_match_device(picoxcell_bus_err_of_match, &pdev->dev);
	drvdata->names = (const char **)id->data;
	dci->mod_name = pdev->dev.driver->name;
	dci->ctl_name = id->compatible;
	dci->dev_name = dev_name(&pdev->dev);

	dci->sysfs_attributes = picoxcell_bus_err_sysfs_attributes;
	drvdata->test_mask = IRQ_ENABLE_ALL;

	if (of_device_is_compatible(pdev->dev.of_node, COMPAT_PC3X3))
		drvdata->test_mask = IRQ_ENABLE_ALL_PC3X3;

	if (edac_device_add_device(dci))
		goto err;

	res = platform_get_irq(pdev, 0);
	if (res < 0)
		goto err2;

	res = devm_request_irq(&pdev->dev, res, picoxcell_bus_err_handler,
			       0, dev_name(&pdev->dev), dci);
	if (res < 0)
		goto err2;

	if (!of_device_is_compatible(pdev->dev.of_node, COMPAT_PC30XX)) {
		res = platform_get_irq(pdev, 1);
		if (res < 0)
			goto err2;

		res = devm_request_irq(&pdev->dev, res,
				       picoxcell_bus_err_handler,
				       0, dev_name(&pdev->dev), dci);
		if (res < 0)
			goto err2;
	}

	/* Ensure that no AXI errors are masked */
	writel(IRQ_MASK_NONE, drvdata->base + IRQ_MASK_OFFSET);

	/* Clear pending interrupt sources and enable all IRQ sources */
	if (of_device_is_compatible(pdev->dev.of_node, COMPAT_PC3X3)) {
		writel(IRQ_CLEAR_ALL_PC3X3,
		       drvdata->base + IRQ_CLEAR_OFFSET);
		writel(IRQ_ENABLE_ALL_PC3X3,
		       drvdata->base + IRQ_ENABLE_OFFSET);
	} else {
		writel(IRQ_CLEAR_ALL,
		       drvdata->base + IRQ_CLEAR_OFFSET);
		writel(IRQ_ENABLE_ALL,
		       drvdata->base + IRQ_ENABLE_OFFSET);
	}

	devres_close_group(&pdev->dev, NULL);
	return 0;
err2:
	edac_device_del_device(&pdev->dev);
err:
	devres_release_group(&pdev->dev, NULL);
	edac_device_free_ctl_info(dci);
	return res;
}

static int picoxcell_bus_err_remove(struct platform_device *pdev)
{
	struct edac_device_ctl_info *dci = platform_get_drvdata(pdev);

	edac_device_del_device(&pdev->dev);
	edac_device_free_ctl_info(dci);
	return 0;
}

MODULE_DEVICE_TABLE(of, picoxcell_bus_err_of_match);

static struct platform_driver picoxcell_bus_err_driver = {
	.probe = picoxcell_bus_err_probe,
	.remove = picoxcell_bus_err_remove,
	.driver = {
		.name		= "picoxcell_bus_err",
		.of_match_table	= picoxcell_bus_err_of_match,
	},
};

module_platform_driver(picoxcell_bus_err_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Michael van der Westhuizen <michael@smart-africa.com>");
MODULE_DESCRIPTION("EDAC Driver for the Picochip picoXcell Bus Error Snoop");
