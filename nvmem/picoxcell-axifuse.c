/*
 * Picochip picoXcell AXIFUSE eFuse Driver
 *
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */
#include <linux/platform_device.h>
#include <linux/nvmem-provider.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define AXIFUSE_NBITS			4096
#define AXIFUSE_NBYTES			512
#define AXIFUSE_CTRL_BYTES		20
#define AXIFUSE_SIZE_BYTES		(AXIFUSE_NBYTES + AXIFUSE_CTRL_BYTES)

#define AXIFUSE_REG_BITS		32
#define AXIFUSE_REG_BYTES		(AXIFUSE_REG_BITS / BITS_PER_BYTE)
#define AXIFUSE_VAL_BITS		32
#define AXIFUSE_VAL_BYTES		(AXIFUSE_VAL_BITS / BITS_PER_BYTE)
#define AXIFUSE_REG_STRIDE		4

#define FUSE_CTRL_OFFSET		0x200
#define FUSE_CTRL_WRITE_BUSY_MASK	(1U << 0)
#define FUSE_CTRL_VDDQ_OE_MASK		(1U << 1)
#define FUSE_CTRL_VDDQ_MASK		(1U << 2)
#define FUSE_WRITE_BIT_ADDRESS_OFFSET	0x204
#define FUSE_WRITE_PERFORM_OFFSET	0x208
#define FUSE_WR_PERFORM_VALUE		0x66757365	/* "fuse" */
#define FUSE_WRITE_PAD_EN_OFFSET	0x20c
#define FUSE_WRITE_PAD_EN_VALUE		0x656e626c	/* "enbl" */
#define FUSE_WRITE_PAD_OFFSET		0x210
#define FUSE_WRITE_PAD_VALUE		0x56444451	/* "VDDQ" */

struct axifuse_protection_range {
	u32 offset;
	u32 nbits;
	u32 bit;
};

struct axifuse_reserved_range {
	u32 offset;
	u32 nbits;
};

struct picoxcell_axifuse_context {
	struct device *dev;
	struct clk *fuse_clk;
	void __iomem *base;
	struct regmap *regmap;
	struct nvmem_device *nvmem;
	struct axifuse_protection_range prot_ranges[10];
	size_t num_prot_ranges;
	struct axifuse_reserved_range rsvd_ranges[4];
	size_t num_rsvd_ranges;
	unsigned long *ltp_cache;
	bool gltp;
};

static int picoxcell_axifuse_write(void *context, const void *data,
				   size_t count)
{
	int ret;
	struct picoxcell_axifuse_context *ctx = context;

	u32 regoff = *(u32 *)data;
	const u32 *buf = data + 4;
	count -= 4;

	dev_info(ctx->dev, "%s: regoff %u, buf %p, count %zu\n", __func__, regoff, buf, count);

	ret = clk_enable(ctx->fuse_clk);
	if (ret)
		return ret;

	/*
	 * TODO:
	 *  -- read existing values
	 *  -- if the register tries to turn off any bits, fail
	 *  -- adjust value to only contain bits that are not already set
	 *  -- mask out reserved values, they can't be written from Linux
	 *  -- if we touch an LTP region, bork
	 *  -- write the values to the fuse IP
	 *  -- write the values to the shadow cache (apparently this can be disabled by fuse - which one?)
	 */

	while (count) {
		dev_info(ctx->dev, "    0x%08x -- 0x%08x\n", 512 - count, *buf);
		count -= AXIFUSE_VAL_BYTES;
		buf++;
	}

	clk_disable(ctx->fuse_clk);

	return 0;
}

static int picoxcell_axifuse_read(void *context, const void *reg,
				  size_t reg_size, void *val,
				  size_t val_size)
{
	int ret;
	struct picoxcell_axifuse_context *ctx = context;
	u32 offset = *(u32 *)reg;
	u32 *buf = val;

	ret = clk_enable(ctx->fuse_clk);
	if (ret)
		return ret;

	while (val_size) {
		*buf++ = readl(ctx->base + offset);
		val_size -= AXIFUSE_VAL_BYTES;
		offset += AXIFUSE_VAL_BYTES;
	}

	clk_disable(ctx->fuse_clk);

	return 0;
}

static bool picoxcell_axifuse_writeable_reg(struct device *dev,
					    unsigned int reg)
{
	struct picoxcell_axifuse_context *ctx = dev_get_drvdata(dev);
	u32 rreg = reg / (AXIFUSE_REG_BITS / 8);
	u32 start = rreg * 32;
	u32 end = start + 32;
	size_t i;

	if (ctx->gltp)
		return false;

	for (i = 0; i < ctx->num_prot_ranges; ++i)
		if (start >= ctx->prot_ranges[i].offset &&
		    end <= ctx->prot_ranges[i].offset +
			   ctx->prot_ranges[i].nbits)
			return !test_bit(ctx->prot_ranges[i].bit,
					 ctx->ltp_cache);

	return true;
}

static struct regmap_bus picoxcell_axifuse_bus = {
	.read = picoxcell_axifuse_read,
	.write = picoxcell_axifuse_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

struct regmap_config picoxcell_axifuse_regmap_config = {
	.reg_bits = AXIFUSE_REG_BITS,
	.val_bits = AXIFUSE_VAL_BITS,
	.reg_stride = AXIFUSE_REG_STRIDE,
	.writeable_reg = picoxcell_axifuse_writeable_reg,
};

static struct nvmem_config econfig = {
	.name = "picoxcell-axifuse",
	.owner = THIS_MODULE,
};

static const struct of_device_id picoxcell_axifuse_match[] = {
	{ .compatible = "picochip,picoxcell-axifuse",},
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, picoxcell_axifuse_match);

#define SET_PROT_RANGE(O, N, B) do {				\
	ctx->prot_ranges[(ctx->num_prot_ranges)].offset = (O);	\
	ctx->prot_ranges[(ctx->num_prot_ranges)].nbits = (N);	\
	ctx->prot_ranges[(ctx->num_prot_ranges)].bit = (B);	\
	ctx->num_prot_ranges++;					\
} while (0)

#define SET_RESVD_RANGE(O, N) do {				\
	ctx->rsvd_ranges[(ctx->num_rsvd_ranges)].offset = (O);	\
	ctx->rsvd_ranges[(ctx->num_rsvd_ranges)].nbits = (N);	\
	ctx->num_rsvd_ranges++;					\
} while (0)

static int axifuse_add_hardware_specifics(struct picoxcell_axifuse_context *ctx)
{
	int ret;
	u32 reg;
	u32 val;
	u32 msk;
	u32 i;

	if (!of_machine_is_compatible("picochip,pc3x3") &&
	    !of_machine_is_compatible("picochip,pc3x2") &&
	    !of_machine_is_compatible("picochip,pc30xx")) {
		return -EINVAL;
	}

	SET_PROT_RANGE(   0,  128, 0);
	SET_PROT_RANGE( 128,  128, 1);
	SET_PROT_RANGE( 256,  128, 2);
	SET_PROT_RANGE( 384,  128, 3);
	SET_PROT_RANGE( 512,  128, 4);
	SET_PROT_RANGE( 640,  128, 5);
	SET_PROT_RANGE( 768,  144, 6);
	SET_PROT_RANGE(1024, 1024, 7);
	SET_PROT_RANGE(2048, 1024, 8);
	SET_PROT_RANGE(3072, 1024, 9);

	if (of_machine_is_compatible("picochip,pc3x3")) {
		SET_RESVD_RANGE( 896, 32);
		SET_RESVD_RANGE( 958, 34);
		SET_RESVD_RANGE(1019,  5);
	} else if (of_machine_is_compatible("picochip,pc3x2")) {
		SET_RESVD_RANGE( 896, 32);
		SET_RESVD_RANGE( 958, 34);
		SET_RESVD_RANGE(1003, 21);
	} else if (of_machine_is_compatible("picochip,pc30xx")) {
		SET_RESVD_RANGE( 904, 24);
		SET_RESVD_RANGE( 958, 34);
		SET_RESVD_RANGE(1019,  2);
		SET_RESVD_RANGE(1022,  2);
	}

	reg = 116;
	ret = picoxcell_axifuse_read(ctx, &reg, sizeof(reg), &val, sizeof(val));
	if (ret)
		return ret;

	for (i = 0; i < 10; ++i) {
		msk = (1U << i);

		if (val & msk)
			set_bit(i, ctx->ltp_cache);
	}

	reg = 118;
	ret = picoxcell_axifuse_read(ctx, &reg, sizeof(reg), &val, sizeof(val));
	if (ret)
		return ret;

	ctx->gltp = !!(val & 0x4);

	return 0;
}

#undef SET_RESVD_RANGE
#undef SET_PROT_RANGE

static int picoxcell_axifuse_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct picoxcell_axifuse_context *ctx;
	int ret;
	u32 nbytes;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = &pdev->dev;

	ctx->fuse_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ctx->fuse_clk))
		return PTR_ERR(ctx->fuse_clk);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource_size(res) != AXIFUSE_SIZE_BYTES)
		return -EINVAL;

	nbytes = AXIFUSE_NBYTES;
	picoxcell_axifuse_regmap_config.max_register = nbytes - 1;

	ctx->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ctx->base))
		return PTR_ERR(ctx->base);

	ctx->regmap = devm_regmap_init(ctx->dev, &picoxcell_axifuse_bus, ctx, &picoxcell_axifuse_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		dev_err(dev, "devm_regmap_init failed\n");
		return PTR_ERR(ctx->regmap);
	}

	ctx->ltp_cache = devm_kzalloc(ctx->dev, BITS_TO_LONGS(10) * sizeof(*ctx->ltp_cache), GFP_KERNEL);
	if (!ctx->ltp_cache)
		return -ENOMEM;

	ret = clk_prepare(ctx->fuse_clk);
	if (ret) {
		return ret;
	}

	ret = axifuse_add_hardware_specifics(ctx);
	if (ret) {
		clk_unprepare(ctx->fuse_clk);
		return ret;
	}

	econfig.dev = ctx->dev;
	ctx->nvmem = nvmem_register(&econfig);
	if (IS_ERR(ctx->nvmem)) {
		dev_err(dev, "nvmem_register failed\n");
		clk_unprepare(ctx->fuse_clk);
		return PTR_ERR(ctx->nvmem);
	}

	platform_set_drvdata(pdev, ctx);

	dev_info(dev, "Registered %u fuses at %p\n", nbytes * 8, ctx->base);

	return 0;
}

static int picoxcell_axifuse_remove(struct platform_device *pdev)
{
	int ret;
	struct picoxcell_axifuse_context *ctx;

	ctx = platform_get_drvdata(pdev);
	if (!ctx)
		return -EINVAL;

	ret = nvmem_unregister(ctx->nvmem);
	if (ret)
		return ret;

	clk_unprepare(ctx->fuse_clk);

	return 0;
}

static struct platform_driver picoxcell_axifuse_driver = {
	.probe = picoxcell_axifuse_probe,
	.remove = picoxcell_axifuse_remove,
	.driver = {
		.name = "picoxcell-axifuse",
		.of_match_table = picoxcell_axifuse_match,
	},
};

module_platform_driver(picoxcell_axifuse_driver);
MODULE_DESCRIPTION("picoXcell AXIFUSE driver");
MODULE_LICENSE("GPL");
