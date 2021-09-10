// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2014-2015, 2017-2019, The Linux Foundation.
 * All rights reserved.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spmi.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#define PMIC_REV2		0x101
#define PMIC_REV3		0x102
#define PMIC_REV4		0x103
#define PMIC_TYPE		0x104
#define PMIC_SUBTYPE		0x105

#define PMIC_TYPE_VALUE		0x51

#define COMMON_SUBTYPE		0x00
#define PM8941_SUBTYPE		0x01
#define PM8841_SUBTYPE		0x02
#define PM8019_SUBTYPE		0x03
#define PM8226_SUBTYPE		0x04
#define PM8110_SUBTYPE		0x05
#define PMA8084_SUBTYPE		0x06
#define PMI8962_SUBTYPE		0x07
#define PMD9635_SUBTYPE		0x08
#define PM8994_SUBTYPE		0x09
#define PMI8994_SUBTYPE		0x0a
#define PM8916_SUBTYPE		0x0b
#define PM8004_SUBTYPE		0x0c
#define PM8909_SUBTYPE		0x0d
#define PM8998_SUBTYPE		0x14
#define PMI8998_SUBTYPE		0x15
#define PM8005_SUBTYPE		0x18

static const struct of_device_id pmic_spmi_id_table[] = {
	{ .compatible = "qcom,spmi-pmic", .data = (void *)COMMON_SUBTYPE },
	{ .compatible = "qcom,pm8941",    .data = (void *)PM8941_SUBTYPE },
	{ .compatible = "qcom,pm8841",    .data = (void *)PM8841_SUBTYPE },
	{ .compatible = "qcom,pm8019",    .data = (void *)PM8019_SUBTYPE },
	{ .compatible = "qcom,pm8226",    .data = (void *)PM8226_SUBTYPE },
	{ .compatible = "qcom,pm8110",    .data = (void *)PM8110_SUBTYPE },
	{ .compatible = "qcom,pma8084",   .data = (void *)PMA8084_SUBTYPE },
	{ .compatible = "qcom,pmi8962",   .data = (void *)PMI8962_SUBTYPE },
	{ .compatible = "qcom,pmd9635",   .data = (void *)PMD9635_SUBTYPE },
	{ .compatible = "qcom,pm8994",    .data = (void *)PM8994_SUBTYPE },
	{ .compatible = "qcom,pmi8994",   .data = (void *)PMI8994_SUBTYPE },
	{ .compatible = "qcom,pm8916",    .data = (void *)PM8916_SUBTYPE },
	{ .compatible = "qcom,pm8004",    .data = (void *)PM8004_SUBTYPE },
	{ .compatible = "qcom,pm8909",    .data = (void *)PM8909_SUBTYPE },
	{ .compatible = "qcom,pm8998",    .data = (void *)PM8998_SUBTYPE },
	{ .compatible = "qcom,pmi8998",   .data = (void *)PMI8998_SUBTYPE },
	{ .compatible = "qcom,pm8005",    .data = (void *)PM8005_SUBTYPE },
	{ }
};

static void pmic_spmi_show_revid(struct regmap *map, struct device *dev)
{
	unsigned int rev2, minor, major, type, subtype;
	const char *name = "unknown";
	int ret, i;

	ret = regmap_read(map, PMIC_TYPE, &type);
	if (ret < 0)
		return;

	if (type != PMIC_TYPE_VALUE)
		return;

	ret = regmap_read(map, PMIC_SUBTYPE, &subtype);
	if (ret < 0)
		return;

	for (i = 0; i < ARRAY_SIZE(pmic_spmi_id_table); i++) {
		if (subtype == (unsigned long)pmic_spmi_id_table[i].data)
			break;
	}

	if (i != ARRAY_SIZE(pmic_spmi_id_table))
		name = pmic_spmi_id_table[i].compatible;

	ret = regmap_read(map, PMIC_REV2, &rev2);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV3, &minor);
	if (ret < 0)
		return;

	ret = regmap_read(map, PMIC_REV4, &major);
	if (ret < 0)
		return;

	/*
	 * In early versions of PM8941 and PM8226, the major revision number
	 * started incrementing from 0 (eg 0 = v1.0, 1 = v2.0).
	 * Increment the major revision number here if the chip is an early
	 * version of PM8941 or PM8226.
	 */
	if ((subtype == PM8941_SUBTYPE || subtype == PM8226_SUBTYPE) &&
	    major < 0x02)
		major++;

	if (subtype == PM8110_SUBTYPE)
		minor = rev2;

	dev_dbg(dev, "%x: %s v%d.%d\n", subtype, name, major, minor);
}

static const struct regmap_config spmi_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= true,
};

static const struct regmap_config spmi_regmap_can_sleep_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0xffff,
	.fast_io	= false,
};

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
struct regmap *g_pm8350b_regmap;
struct device *g_pm8350b_dev;
#define pm8350b_reg_dump_PROC_FILE	"driver/pm8350b_reg_dump"
static struct proc_dir_entry *pm8350b_reg_dump_proc_file;
static int pm8350b_reg_dump_proc_read(struct seq_file *buf, void *v)
{
	unsigned int val;
	int ret;
	int i;
	unsigned int address1 = 0x2600;
	unsigned int address2 = 0x4800;
	unsigned int address3 = 0x4F00;
	unsigned int address4 = 0xF200;

	for (i = 0; i <= 1791; i++) {
		ret = regmap_read(g_pm8350b_regmap, address1, &val);
		if (ret < 0)
			seq_printf(buf, "%x: XX\n", address1);
		else
			seq_printf(buf, "%x: %x\n", address1, val);

		address1 = address1 + 0x1;
	}

	for (i = 0; i <= 255; i++) {
		ret = regmap_read(g_pm8350b_regmap, address2, &val);
		if (ret < 0)
			seq_printf(buf, "%x: XX\n", address2);
		else
			seq_printf(buf, "%x: %x\n", address2, val);

		address2 = address2 + 0x1;
	}

	for (i = 0; i <= 255; i++) {
		ret = regmap_read(g_pm8350b_regmap, address3, &val);
		if (ret < 0)
			seq_printf(buf, "%x: XX\n", address3);
		else
			seq_printf(buf, "%x: %x\n", address3, val);

		address3 = address3 + 0x1;
	}

	for (i = 0; i <= 255; i++) {
		ret = regmap_read(g_pm8350b_regmap, address4, &val);
		if (ret < 0)
			seq_printf(buf, "%x: XX\n", address4);
		else
			seq_printf(buf, "%x: %x\n", address4, val);

		address4 = address4 + 0x1;
	}

	return 0;
}

static int pm8350b_reg_dump_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, pm8350b_reg_dump_proc_read, NULL);
}

static const struct file_operations pm8350b_reg_dump_fops = {
	.owner = THIS_MODULE,
    .open = pm8350b_reg_dump_proc_open,
    .read = seq_read,
    .release = single_release,
};

void static create_pm8350b_reg_dump_proc_file(void)
{
	pm8350b_reg_dump_proc_file = proc_create(pm8350b_reg_dump_PROC_FILE, 0644, NULL, &pm8350b_reg_dump_fops);

	if (pm8350b_reg_dump_proc_file) {
		dev_err(g_pm8350b_dev, "[PMIC_REG_DUMP]: create_pm8350b_reg_dump_proc_file sucessed!\n");
	} else {
		dev_err(g_pm8350b_dev, "[PMIC_REG_DUMP]: create_pm8350b_reg_dump_proc_file failed!\n");
	}
}

struct regmap *g_smb1396_regmap;
struct device *g_smb1396_dev;
#define smb1396_reg_dump_PROC_FILE	"driver/smb1396_reg_dump"
static struct proc_dir_entry *smb1396_reg_dump_proc_file;
static int smb1396_reg_dump_proc_read(struct seq_file *buf, void *v)
{
	unsigned int val;
	int ret;
	int i;
	unsigned int address = 0x2600;

	for (i = 0; i <= 1791; i++) {
		ret = regmap_read(g_smb1396_regmap, address, &val);
		if (ret < 0)
			seq_printf(buf, "%x: XX\n", address);
		else
			seq_printf(buf, "%x: %x\n", address, val);

		address = address + 0x1;
	}

	return 0;
}

static int smb1396_reg_dump_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, smb1396_reg_dump_proc_read, NULL);
}

static const struct file_operations smb1396_reg_dump_fops = {
	.owner = THIS_MODULE,
    .open = smb1396_reg_dump_proc_open,
    .read = seq_read,
    .release = single_release,
};

void static create_smb1396_reg_dump_proc_file(void)
{
	smb1396_reg_dump_proc_file = proc_create(smb1396_reg_dump_PROC_FILE, 0644, NULL, &smb1396_reg_dump_fops);

	if (smb1396_reg_dump_proc_file) {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: create_smb1396_reg_dump_proc_file sucessed!\n");
	} else {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: create_smb1396_reg_dump_proc_file failed!\n");
	}
}

#define smb1396_en_PROC_FILE	"driver/smb1396_en"
static struct proc_dir_entry *smb1396_en_proc_file;
static int smb1396_en_proc_read(struct seq_file *buf, void *v)
{
	unsigned int val = 0;
	int ret;
	unsigned int address = 0x2641;

	ret = regmap_read(g_smb1396_regmap, address, &val);
	if (ret < 0) {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: smb1396_en regmap_read failed\n");
		seq_printf(buf, "%d\n", -ENODEV);
	} else {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: smb1396_en bit[7] = 0x%x\n", val);
		if (val & 0x80)
			seq_printf(buf, "%d\n", true);
		 else
			seq_printf(buf, "%d\n", false);
	}

	return 0;
}

static int smb1396_en_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, smb1396_en_proc_read, NULL);
}

static const struct file_operations smb1396_en_fops = {
	.owner = THIS_MODULE,
    .open = smb1396_en_proc_open,
    .read = seq_read,
    .release = single_release,
};

void static create_smb1396_en_proc_file(void)
{
	smb1396_en_proc_file = proc_create(smb1396_en_PROC_FILE, 0644, NULL, &smb1396_en_fops);

	if (smb1396_en_proc_file) {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: create_smb1396_en_proc_file sucessed!\n");
	} else {
		dev_err(g_smb1396_dev, "[PMIC_REG_DUMP]: create_smb1396_en_proc_file failed!\n");
	}
}
#endif

static int pmic_spmi_probe(struct spmi_device *sdev)
{
	struct device_node *root = sdev->dev.of_node;
	struct regmap *regmap;

	if (of_property_read_bool(root, "qcom,can-sleep"))
		regmap = devm_regmap_init_spmi_ext(sdev,
						&spmi_regmap_can_sleep_config);
	else
		regmap = devm_regmap_init_spmi_ext(sdev, &spmi_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	/* Only the first slave id for a PMIC contains this information */
	if (sdev->usid % 2 == 0)
		pmic_spmi_show_revid(regmap, &sdev->dev);

#if defined ASUS_ZS673KS_PROJECT || defined ASUS_PICASSO_PROJECT
	if (sdev->usid == 0x3 && pm8350b_reg_dump_proc_file == NULL) {
		dev_err(&sdev->dev, "[PMIC_REG_DUMP]: pm8350b name:%s\n", dev_name(&sdev->dev));
		g_pm8350b_regmap = regmap;
		g_pm8350b_dev = &sdev->dev;
		create_pm8350b_reg_dump_proc_file();
	}

	if (sdev->usid == 0xb && smb1396_reg_dump_proc_file == NULL) {
		dev_err(&sdev->dev, "[PMIC_REG_DUMP]: smb1396 name:%s\n", dev_name(&sdev->dev));
		g_smb1396_regmap = regmap;
		g_smb1396_dev = &sdev->dev;
		create_smb1396_reg_dump_proc_file();		
		create_smb1396_en_proc_file();
	}
#endif

	return devm_of_platform_populate(&sdev->dev);
}

MODULE_DEVICE_TABLE(of, pmic_spmi_id_table);

static struct spmi_driver pmic_spmi_driver = {
	.probe = pmic_spmi_probe,
	.driver = {
		.name = "pmic-spmi",
		.of_match_table = pmic_spmi_id_table,
	},
};

static int __init pmic_spmi_init(void)
{
	return spmi_driver_register(&pmic_spmi_driver);
}
arch_initcall(pmic_spmi_init);

static void __exit pmic_spmi_exit(void)
{
	spmi_driver_unregister(&pmic_spmi_driver);
}
module_exit(pmic_spmi_exit);

MODULE_DESCRIPTION("Qualcomm SPMI PMIC driver");
MODULE_ALIAS("spmi:spmi-pmic");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Josh Cartwright <joshc@codeaurora.org>");
MODULE_AUTHOR("Stanimir Varbanov <svarbanov@mm-sol.com>");
