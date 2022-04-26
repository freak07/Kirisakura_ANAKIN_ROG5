/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	struct pinctrl *fp_pc;
	struct pinctrl_state *fp_pcs;

	printk("[FP][%s] +++\n", __func__);

// ASUS BSP Pin Ctrl +++
	fp_pc = devm_pinctrl_get(&gf_dev->spi->dev);
	if (IS_ERR_OR_NULL(fp_pc)) {
		pr_err("[GF] failed to get fp pinctrl\n");
	}
	fp_pcs = pinctrl_lookup_state(fp_pc, "fp_default");
	if (IS_ERR_OR_NULL(fp_pcs)) {
		pr_err("[GF] failed to get fp pinctrl state from dtsi\n");
	}
	rc = pinctrl_select_state(fp_pc, fp_pcs);
	if (rc < 0) {
		pr_err("[GF] failed to set fp pinctrl state\n");
	}
// ASUS BSP Pin Ctrl ---

	gf_dev->reset_gpio = of_get_named_gpio(np, "fp-gpio-reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_reset;
	}
	gpio_direction_output(gf_dev->reset_gpio, 0);

	gf_dev->irq_gpio = of_get_named_gpio(np, "fp-gpio-irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);

err_irq:
	devm_gpio_free(dev, gf_dev->reset_gpio);
err_reset:
	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;
	u32 voltage_supply[2];

	pr_info("[FP][%s] start.\n", __func__);

	/* TODO: add your power control here */
	/* asus bsp add for pwr ldo control +++ */
	rc = of_property_read_u32_array(np, "asus-fp,vcc-voltage", voltage_supply, 2);
    if (rc < 0) {
    	printk("[FP][%s] Failed to get regulator avdd voltage !\n", __func__);
		return rc;
    }
	printk("[FP][%s] Regulator voltage get Max = %d, Min = %d \n", __func__, voltage_supply[1], voltage_supply[0]);
	gf_dev->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR( gf_dev->vcc)) {
		rc = PTR_ERR(gf_dev->vcc);
		printk("[FP][%s] Regulator get vcc failed rc=%d\n", __func__, rc);
		goto reg_vdd_get_vtg;
	}
	if (regulator_count_voltages(gf_dev->vcc) > 0) {
		rc = regulator_set_voltage(gf_dev->vcc, voltage_supply[0],  voltage_supply[1]);
		if (rc) {
			printk("[FP][%s] Regulator set_vcc failed vdd rc=%d\n", __func__, rc);
			goto reg_vcc_i2c_put;
		}
	}
	
	rc = regulator_enable(gf_dev->vcc);
	if (rc) {
	  printk("[FP][%s]Regulator vcc enable failed rc=%d\n", __func__, rc);
	} else {
	  printk("[FP][%s]Regulator vcc enable ! \n", __func__);
	}
	/* asus bsp add for pwr ldo control --- */

	mdelay(15);

	if (gpio_is_valid(gf_dev->reset_gpio)) {
		rc = gpio_direction_output(gf_dev->reset_gpio, 1);
		if (rc)
			printk("[FP][%s] reset gpio enable failed, rc = %d\n", __func__, rc);
		else
			printk("[FP][%s] reset gpio enable !\n", __func__);
	}

	mdelay(3);

	pr_info("[FP][%s] end.\n", __func__);
	return rc;

reg_vcc_i2c_put:
    regulator_put(gf_dev->vcc);
reg_vdd_get_vtg:
	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;

	/* TODO: add your power control here */

	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

