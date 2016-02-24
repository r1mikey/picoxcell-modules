/*
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/regmap.h>

#include "config_port.h"

#define SDGPIO_MAX_PORTS		2
#define SDGPIO_MAX_PINS			32
/* The base address of SD-GPIO config registers in the AXI2Pico. */
#define SD_PIN_CONFIG_BASE		0x9800
/* The spacing between SD-GPIO config registers. */
#define SD_PIN_CONFIG_SPACING		4
/* PICOXCELL AXI2Pico CAEID. */
#define PICOXCELL_AXI2PICO_CAEID	0x9000
/* Soft reset lock bit. */
#define SD_CONFIG_SR_LOCK		(1 << 13)
/* Control source bit. */
#define SD_CONFIG_CS_MASK		(~(1 << 15))
/* The mask for analogue converter size in the config register. */
#define SD_CONV_SZ_MASK			0xF
/* Analogue not digital bit. */
#define SD_CONFIG_AND			(1 << 14)
/* The address of the control value register in the AXI2Pico. */
#define SD_CONTROL_VAL_REG		0x9882
/* The base address of SD-GPIO analogue rate registers in the AXI2Pico. */
#define SD_PIN_ANALOGUE_RATE_BASE	0x9802
/* The base address of SD-GPIO analogue value registers in the AXI2Pico. */
#define SD_PIN_ANALOGUE_VALUE_BASE	0x9801
/* The address of the output value register in the AXI2Pico. */
#define SD_OUTPUT_VAL_REG		0x9884
/* The address of the output value high register in the AXI2Pico (pc3x3). */
#define SD_OUTPUT_HI_VAL_REG		0x9885
/* The address of the input value register in the AXI2Pico. */
#define SD_INPUT_VAL_REG		0x9880
/* The address of the input value high register in the AXI2Pico (pc3x3). */
#define SD_INPUT_VAL_HI_REG		0x9880

struct sdgpio_port_property {
	struct device_node *node;
	const char *name;
	u16 ngpio;
	int gpio_base;
	u16 sd_base;
};

struct sdgpio_platform_data {
	struct platform_device *cfgport;
	struct regmap *shd;
	struct sdgpio_port_property *properties;
	unsigned int nports;
	unsigned int ngpios;
};

struct sdgpio_gpio;

struct sdgpio_chip {
	struct gpio_chip gc;
	bool is_registered;
	struct sdgpio_gpio *gpio;
	u16 sd_base;
};

struct sdgpio_gpio {
	struct device *dev;
	struct device *cfgport;
	struct regmap *shd;
	struct sdgpio_chip *ports;
	unsigned int nr_ports;
	unsigned int nr_gpios;
	unsigned long *a_not_d_map;
	spinlock_t lock;
};

static inline struct sdgpio_chip *to_sdgpio_chip(struct gpio_chip *gc)
{
	return container_of(gc, struct sdgpio_chip, gc);
}

static inline int sdgpio_block_offset(struct sdgpio_chip *sc, int offset)
{
	return sc->sd_base + offset;
}

/*
 * Get the address of a config register for a pin
 *
 * sc: The SDGPIO chip instance
 * offset: The offset of the pin in chip-space
 */
static inline u16 sd_pin_config(struct sdgpio_chip *sc, unsigned offset)
{
	return SD_PIN_CONFIG_BASE + (sdgpio_block_offset(sc, offset) * SD_PIN_CONFIG_SPACING);
}

/*
 * Get the address of an analogue rate register for a pin
 */
static inline u16 sd_pin_analogue_rate(struct sdgpio_chip *sc, unsigned offset)
{
	return SD_PIN_ANALOGUE_RATE_BASE + (sdgpio_block_offset(sc, offset) * SD_PIN_CONFIG_SPACING);
}

/*
 * Get the address of an analogue value register for a pin
 */
static inline u16 sd_pin_analogue_value(struct sdgpio_chip *sc, unsigned offset)
{
	return SD_PIN_ANALOGUE_VALUE_BASE + (sdgpio_block_offset(sc, offset) * SD_PIN_CONFIG_SPACING);
}

static int sdgpio_reset_config(struct sdgpio_chip *sc, unsigned offset,
			       int value)
{
	int ret;
	u16 data;

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 sd_pin_config(sc, offset),
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read config register for SDGPIO %u\n",
			sdgpio_block_offset(sc, offset));
		return ret < 0 ? ret : -EIO;
	}

	if (value)
		data |= SD_CONFIG_SR_LOCK;
	else
		data &= ~SD_CONFIG_SR_LOCK;

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    sd_pin_config(sc, offset), &data, 1);
	return 0;
}

static int configure_dac(struct sdgpio_chip *sc, unsigned offset, u8 converter_size, u16 analogue_rate)
{
	int ret;
	u16 data;

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 sd_pin_config(sc, offset),
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read config register for SDGPIO %u\n",
			sdgpio_block_offset(sc, offset));
		return ret < 0 ? ret : -EIO;
	}

	data &= SD_CONFIG_CS_MASK | ~SD_CONV_SZ_MASK;

	if (!analogue_rate && !converter_size)
		data &= ~SD_CONFIG_AND;
	else
		data |= SD_CONFIG_AND;

	data |= (converter_size & SD_CONV_SZ_MASK);

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    sd_pin_config(sc, offset), &data, 1);

	/*
	 * Configure the pin to drive the output
	 */
	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 SD_CONTROL_VAL_REG,
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read SDGPIO control value register\n");
		return ret < 0 ? ret : -EIO;
	}

	data |= (1U << sdgpio_block_offset(sc, offset));

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    SD_CONTROL_VAL_REG,
				    &data, 1);

	/*
	 * Write the analogue rate register
	 */
	data = analogue_rate;
	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    sd_pin_analogue_rate(sc, offset),
				    &data, 1);

	if (analogue_rate || converter_size)
		set_bit(sdgpio_block_offset(sc, offset), sc->gpio->a_not_d_map);
	else
		clear_bit(sdgpio_block_offset(sc, offset), sc->gpio->a_not_d_map);

	return 0;
}

int picoxcell_gpio_configure_dac(struct gpio_desc *desc, u8 converter_size, u16 analogue_rate)
{
	struct gpio_chip *chip;
	struct sdgpio_chip *sc;
	int pin;
	int ret;
	unsigned long flags;

	chip = gpiod_to_chip(desc);
	if (!chip)
		return -EINVAL;

	sc = to_sdgpio_chip(chip);
	if (!sc)
		return -EINVAL;

	pin = desc_to_gpio(desc);
	if (pin < chip->base || pin > chip->base + chip->ngpio)
		return -EINVAL;

	spin_lock_irqsave(&sc->gpio->lock, flags);
	ret = configure_dac(sc, pin - chip->base, converter_size, analogue_rate);
	spin_unlock_irqrestore(&sc->gpio->lock, flags);
	return ret;
}
EXPORT_SYMBOL_GPL(picoxcell_gpio_configure_dac);

static int sdgpio_get_digital_out_status(struct sdgpio_chip *sc, u32 *v)
{
	int ret;
	u16 data[2] = { 0, 0 };

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 SD_OUTPUT_VAL_REG,
					 &data[0], 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read digital out status register\n");
		return ret < 0 ? ret : -EIO;
	}

	if (sc->gpio->nr_gpios > 16) {
		ret = picoxcell_config_port_read(sc->gpio->cfgport,
						 PICOXCELL_AXI2PICO_CAEID,
						 SD_OUTPUT_HI_VAL_REG,
						 &data[1], 1);
		if (ret != 1) {
			dev_err(sc->gpio->dev,
				"failed to read digital out high status register\n");
			return ret < 0 ? ret : -EIO;
		}
	}

	*v = data[0] | (data[1] << 16);
	return 0;
}

static void sdgpio_set_digital_out_status(struct sdgpio_chip *sc, u32 v)
{
	u16 data[2] = { (u16)(v & 0xFFFF), (u16)((v >> 16) & 0xFFFF) };

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    SD_OUTPUT_VAL_REG, &data[0], 1);

	if (sc->gpio->nr_gpios > 16) {
		picoxcell_config_port_write(sc->gpio->cfgport,
					    PICOXCELL_AXI2PICO_CAEID,
					    SD_OUTPUT_HI_VAL_REG,
					    &data[1], 1);
	}
}

static int sdgpio_get_digital_in_status(struct sdgpio_chip *sc, u32 *v)
{
	int ret;
	u16 data[2] = { 0, 0 };

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 SD_INPUT_VAL_REG,
					 &data[0], 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read digital in status register\n");
		return ret < 0 ? ret : -EIO;
	}


	if (sc->gpio->nr_gpios > 16) {
		ret = picoxcell_config_port_read(sc->gpio->cfgport,
						 PICOXCELL_AXI2PICO_CAEID,
						 SD_INPUT_VAL_HI_REG,
						 &data[1], 1);
		if (ret != 1) {
			dev_err(sc->gpio->dev,
				"failed to read digital in high status register\n");
			return ret < 0 ? ret : -EIO;
		}
	}

	*v = data[0] | (data[1] << 16);
	return 0;
}

static int sdgpio_set_direction(struct sdgpio_chip *sc, unsigned offset, bool input)
{
	int ret;
	u16 data;

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 sd_pin_config(sc, offset),
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read config register for SDGPIO %u\n",
			sdgpio_block_offset(sc, offset));
		return ret < 0 ? ret : -EIO;
	}

	data &= SD_CONFIG_CS_MASK;

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    sd_pin_config(sc, offset),
				    &data, 1);

	/*
	 * Configure the pin to drive or not drive the output as appropriate
	 */
	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 SD_CONTROL_VAL_REG,
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev,
			"failed to read SDGPIO control value register\n");
		return ret < 0 ? ret : -EIO;
	}

	if (input)
		data &= ~(1U << sdgpio_block_offset(sc, offset));
	else
		data |= (1U << sdgpio_block_offset(sc, offset));

	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    SD_CONTROL_VAL_REG,
				    &data, 1);
	return 0;
}

/*
 * Configure or deconfigure a pin for use in the SD domain by manipulating
 * the shared mux register.
 *
 * When clear the pin is under SD control, when set it is under ARM control.
 */
static inline int sdgpio_set_clear_shd_bit(struct sdgpio_chip *sc,
					   unsigned offset,
					   bool set)
{
	u32 mask = (1U << sdgpio_block_offset(sc, offset));
	u32 val = set ? mask : 0;
	return regmap_update_bits(sc->gpio->shd, 0x0, mask, val);
}

static int sdgpio_request(struct gpio_chip *chip, unsigned offset)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	int ret;
	unsigned long flags;

	ret = pinctrl_request_gpio(chip->base + offset);
	if (ret)
		return ret;

	ret = sdgpio_set_clear_shd_bit(sc, offset, false);
	if (ret) {
		pinctrl_free_gpio(chip->base + offset);
		return ret;
	}

	spin_lock_irqsave(&sc->gpio->lock, flags);
	ret = sdgpio_reset_config(sc, offset, 1);
	spin_unlock_irqrestore(&sc->gpio->lock, flags);

	return ret;
}

static void sdgpio_free(struct gpio_chip *chip, unsigned offset)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	unsigned long flags;

	spin_lock_irqsave(&sc->gpio->lock, flags);
	configure_dac(sc, offset, 0, 0);
	spin_unlock_irqrestore(&sc->gpio->lock, flags);
	(void)sdgpio_set_clear_shd_bit(sc, offset, true);
	pinctrl_free_gpio(chip->base + offset);
}

/*
 * Return the analogue value for a pin configured as an analogue input.
 *
 * Must be called with the SDGPIO device lock held.
 */
static int sdgpio_get_analogue(struct sdgpio_chip *sc, unsigned offset)
{
	u16 data;
	int ret;

	ret = picoxcell_config_port_read(sc->gpio->cfgport,
					 PICOXCELL_AXI2PICO_CAEID,
					 sd_pin_analogue_value(sc, offset),
					 &data, 1);
	if (ret != 1) {
		dev_err(sc->gpio->dev, "failed to read the analogue "
				       "value register for SDGPIO pin %u\n",
			sdgpio_block_offset(sc, offset));
		return ret < 0 ? ret : -EIO;
	}

	return (int)data;
}

/*
 * Return the digital value for a pin configured as an digital input.
 *
 * Must be called with the SDGPIO device lock held.
 */
static int sdgpio_get_digital(struct sdgpio_chip *sc, unsigned offset)
{
	u32 status;
	int ret;

	ret = sdgpio_get_digital_in_status(sc, &status);
	if (ret)
		return ret < 0 ? ret : -EIO;

	return !!(status & (1U << sdgpio_block_offset(sc, offset)));
}

static int sdgpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&sc->gpio->lock, flags);

	if (test_bit(sdgpio_block_offset(sc, offset), sc->gpio->a_not_d_map))
		ret = sdgpio_get_analogue(sc, offset);
	else
		ret = sdgpio_get_digital(sc, offset);

	spin_unlock_irqrestore(&sc->gpio->lock, flags);

	return ret;
}

/*
 * Sets an analogue value to be driven on a GPIO.
 *
 * Must be called with the SDGPIO device lock held.
 */
static void sdgpio_set_analogue(struct sdgpio_chip *sc, unsigned offset,
				int value)
{
	u16 data = (u16)value;
	picoxcell_config_port_write(sc->gpio->cfgport,
				    PICOXCELL_AXI2PICO_CAEID,
				    sd_pin_analogue_value(sc, offset),
				    &data, 1);
}

/*
 * Sets an digital value to be driven on a GPIO.
 *
 * Must be called with the SDGPIO device lock held.
 */
static void sdgpio_set_digital(struct sdgpio_chip *sc, unsigned offset,
			       int value)
{
	int ret;
	u32 status;

	ret = sdgpio_get_digital_out_status(sc, &status);
	if (ret) {
		dev_err(sc->gpio->dev,
			"failed to read SDGPIO output value register\n");
		return;
	}

	status &= ~(1U << sdgpio_block_offset(sc, offset));
	status |= (!!value) << sdgpio_block_offset(sc, offset);

	sdgpio_set_digital_out_status(sc, status);
}

/*
 * Sets a value to be driven on a GPIO.
 * The value driven will be analogue or digital based on the configuration
 * of the pin.
 *
 * Must be called with the SDGPIO device lock held.
 */
static void sdgpio_set_unlocked(struct sdgpio_chip *sc, unsigned offset,
				int value)
{
	if (test_bit(sdgpio_block_offset(sc, offset), sc->gpio->a_not_d_map))
		sdgpio_set_analogue(sc, offset, value);
	else
		sdgpio_set_digital(sc, offset, value);
}

static void sdgpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	unsigned long flags;

	spin_lock_irqsave(&sc->gpio->lock, flags);
	sdgpio_set_unlocked(sc, offset, value);
	spin_unlock_irqrestore(&sc->gpio->lock, flags);
}

static int sdgpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sc->gpio->lock, flags);
	ret = sdgpio_set_direction(sc, offset, true);
	spin_unlock_irqrestore(&sc->gpio->lock, flags);

	return ret;
}

static int sdgpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct sdgpio_chip *sc = to_sdgpio_chip(chip);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&sc->gpio->lock, flags);

	ret = sdgpio_set_direction(sc, offset, false);
	if (!ret)
		sdgpio_set_unlocked(sc, offset, value);

	spin_unlock_irqrestore(&sc->gpio->lock, flags);

	return ret;
}

static int sdgpio_add_port(struct sdgpio_gpio *gpio,
			   struct sdgpio_port_property *pp,
			   unsigned int offs)
{
	struct sdgpio_chip *port;
	int err;

	port				= &gpio->ports[offs];
	port->gpio			= gpio;
	port->sd_base			= pp->sd_base;

#ifdef CONFIG_OF_GPIO
	port->gc.of_node		= pp->node;
#endif
	port->gc.parent			= gpio->dev;
	port->gc.label			= dev_name(gpio->dev);
	port->gc.base			= pp->gpio_base;
	port->gc.ngpio			= pp->ngpio;

	port->gc.request		= sdgpio_request;
	port->gc.free			= sdgpio_free;
	port->gc.direction_input	= sdgpio_direction_input;
	port->gc.direction_output	= sdgpio_direction_output;
	port->gc.get			= sdgpio_get;
	port->gc.set			= sdgpio_set;

	err = gpiochip_add(&port->gc);
	if (err)
		dev_err(gpio->dev, "failed to register gpiochip for %s\n",
			pp->name);
	else {
		port->is_registered = true;
		gpio->nr_gpios += port->gc.ngpio;
		dev_info(gpio->dev,
			 "Registered SDGPIO chip %s at %d with %u GPIOs\n",
			 port->gc.label, port->gc.base, port->gc.ngpio);
	}

	return err;
}

static void sdgpio_unregister(struct sdgpio_gpio *gpio)
{
	unsigned int m;

	for (m = 0; m < gpio->nr_ports; ++m)
		if (gpio->ports[m].is_registered)
			gpiochip_remove(&gpio->ports[m].gc);
}

static struct sdgpio_platform_data * sdgpio_get_pdata_of(struct device *dev)
{
	struct device_node *node, *port_np;
	struct sdgpio_platform_data *pdata;
	struct sdgpio_port_property *pp;
	struct device_node *cfgport_node;
	struct platform_device *cfgport_pdev;
	struct regmap *shd;
	int nports;
	int i;
	u16 tmp;

	node = dev->of_node;
	if (!IS_ENABLED(CONFIG_OF_GPIO) || !node)
		return ERR_PTR(-ENODEV);

	cfgport_node = of_parse_phandle(node, "picochip,cfgport", 0);
	if (!cfgport_node)
		return ERR_PTR(-EPROBE_DEFER);

	cfgport_pdev = of_find_device_by_node(cfgport_node);
	if (!cfgport_pdev)
		return ERR_PTR(-EPROBE_DEFER);

	shd = syscon_regmap_lookup_by_phandle(node, "picochip,shdmux");
	if (IS_ERR(shd))
		return ERR_PTR(PTR_ERR(shd));

	nports = of_get_child_count(node);
	if (nports == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->cfgport = cfgport_pdev;
	pdata->shd = shd;

	pdata->properties = devm_kcalloc(dev, nports, sizeof(*pp), GFP_KERNEL);
	if (!pdata->properties)
		return ERR_PTR(-ENOMEM);

	pdata->nports = nports;

	i = 0;
	for_each_child_of_node(node, port_np) {
		pp = &pdata->properties[i++];
		pp->node = port_np;
		pp->name = port_np->full_name;

		if (of_property_read_u16(port_np, "picochip,nr-gpios",
					 &pp->ngpio)) {
			dev_err(dev,
				 "failed to get number of gpios for %s\n",
				 port_np->full_name);
			return ERR_PTR(-EINVAL);
		}

		if (of_property_read_u16(port_np, "picochip,sdgpio-base",
					 &pp->sd_base)) {
			dev_err(dev,
				 "failed to get number of SD base for %s\n",
				 port_np->full_name);
			return ERR_PTR(-EINVAL);
		}

		if (of_property_read_u16(port_np, "picochip,gpio-base",
					 &tmp))
			pp->gpio_base = -1;
		else
			pp->gpio_base = (int)tmp;
	}

	return pdata;
}

static int sdgpio_probe(struct platform_device *pdev)
{
	int err;
	int i;
	struct sdgpio_gpio *gpio;
	struct device *dev = &pdev->dev;
	struct sdgpio_platform_data *pdata = dev_get_platdata(dev);

	if (!pdata) {
		pdata = sdgpio_get_pdata_of(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	if (!pdata->nports || !pdata->cfgport)
		return -ENODEV;

	if (pdata->nports > SDGPIO_MAX_PORTS)
		return -EINVAL;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	spin_lock_init(&gpio->lock);
	gpio->dev = &pdev->dev;
	gpio->shd = pdata->shd;
	gpio->nr_ports = pdata->nports;

	gpio->ports = devm_kcalloc(&pdev->dev, gpio->nr_ports,
				   sizeof(*gpio->ports), GFP_KERNEL);
	if (!gpio->ports)
		return -ENOMEM;

	gpio->cfgport = &pdata->cfgport->dev;

	if (!try_module_get(gpio->cfgport->driver->owner)) {
		err = -ENODEV;
		goto out_unregister;
	}

	for (i = 0; i < gpio->nr_ports; ++i) {
		err = sdgpio_add_port(gpio, &pdata->properties[i], i);
		if (err)
			goto out_putcfgport;
	}

	if (gpio->nr_gpios > SDGPIO_MAX_PINS) {
		dev_err(dev, "This driver supports no more than %u SDGPIO pins per picoArray instance\n", SDGPIO_MAX_PINS);
		return -EINVAL;
	}

	gpio->a_not_d_map = devm_kzalloc(dev, BITS_TO_LONGS(gpio->nr_gpios) * sizeof(*gpio->a_not_d_map), GFP_KERNEL);
	if (!gpio->a_not_d_map) {
		err = -ENOMEM;
		goto out_putcfgport;
	}

	platform_set_drvdata(pdev, gpio);
	dev_info(dev, "Registered %u GPIOs on %s\n", gpio->nr_gpios,
		 dev_name(gpio->cfgport));
	return 0;

out_putcfgport:
	module_put(gpio->cfgport->driver->owner);

out_unregister:
	sdgpio_unregister(gpio);
	return err;
}

static int sdgpio_remove(struct platform_device *pdev)
{
	struct sdgpio_gpio *gpio = platform_get_drvdata(pdev);

	module_put(gpio->cfgport->driver->owner);
	sdgpio_unregister(gpio);
	return 0;
}

static const struct of_device_id sdgpio_of_match[] = {
	{ .compatible = "picochip,picoxcell-sdgpio" },
	{}
};
MODULE_DEVICE_TABLE(of, sdgpio_of_match);

static struct platform_driver sdgpio_driver = {
	.driver	= {
		.name		= "gpio-sdgpio",
		/* .pm		= &sdgpio_pm_ops, */
		.of_match_table	= of_match_ptr(sdgpio_of_match),
	},
	.probe	= sdgpio_probe,
	.remove	= sdgpio_remove,
};

module_platform_driver(sdgpio_driver);

MODULE_AUTHOR("Jamie Iles <jamie@jamieiles.com>, "
	      "Michael van der Westhuizen <michael@smart-africa.com>");
MODULE_DESCRIPTION("Picochip picoXcell Sigma-Delta GPIO driver");
MODULE_LICENSE("GPL");
