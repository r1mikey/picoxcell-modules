/*
 * Copyright (c) 2010 Picochip Ltd., Jamie Iles
 * Copyright (c) 2015, Michael van der Westhuizen <michael@smart-africa.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _PICOXCELL_CONFIG_PORT_H
#define _PICOXCELL_CONFIG_PORT_H

#include <linux/device.h>
#include <linux/types.h>

extern int picoxcell_config_port_read(struct device *dev, u16 aeid, u16 ae_addr,
                        u16 *buf, u16 count);

extern void picoxcell_config_port_write(struct device *dev, u16 aeid, u16 ae_addr,
                          const u16 *buf, u16 count);

extern void picoxcell_config_port_write_buf(struct device *dev, const u32 *buf, unsigned nr_words);

#endif
