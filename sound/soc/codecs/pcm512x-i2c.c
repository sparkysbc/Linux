/*
 * Driver for the PCM512x CODECs
 *
 * Author:	Mark Brown <broonie@linaro.org>
 *		Copyright 2014 Linaro Ltd
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include "pcm512x.h"

#if 0
#include <linux/24c32.h>
#endif 
static int pcm512x_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct regmap *regmap;
	struct regmap_config config = pcm512x_regmap;
#if 0
	if(PCM5122_ID == dac_id) {
	
	//int ret = 0;
	//ret = cat24c32_read_id();	
	//printk("\n CEM ret : 0x%x ",ret);
	//if (PCM5122_ID == ret) {
#endif	
	/* msb needs to be set to enable auto-increment of addresses */
	config.read_flag_mask = 0x80;
	config.write_flag_mask = 0x80;

	regmap = devm_regmap_init_i2c(i2c, &config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);
	printk("PCM512x_i2c_probe...*************  addr: %x \n",i2c->addr);
	return pcm512x_probe(&i2c->dev, regmap);
#if 0
	} else {
	 return 0;
	}
#endif 
}

static int pcm512x_i2c_remove(struct i2c_client *i2c)
{
	pcm512x_remove(&i2c->dev);
	printk("PCM512x_i2c_remove over addr: %x \n",i2c->addr);
	return 0;
}

static const struct i2c_device_id pcm512x_i2c_id[] = {
	{ "pcm5121", },
	{ "pcm5122", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcm512x_i2c_id);

static const struct of_device_id pcm512x_of_match[] = {
	{ .compatible = "ti,pcm5121", },
	{ .compatible = "ti,pcm5122", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm512x_of_match);

static struct i2c_driver pcm512x_i2c_driver = {
	.probe 		= pcm512x_i2c_probe,
	.remove 	= pcm512x_i2c_remove,
	.id_table	= pcm512x_i2c_id,
	.driver		= {
		.name	= "pcm512x",
		.owner	= THIS_MODULE,
		.of_match_table = pcm512x_of_match,
		.pm     = &pcm512x_pm_ops,
	},
};

module_i2c_driver(pcm512x_i2c_driver);

MODULE_DESCRIPTION("ASoC PCM512x codec driver - I2C");
MODULE_AUTHOR("Mark Brown <broonie@linaro.org>");
MODULE_LICENSE("GPL v2");
