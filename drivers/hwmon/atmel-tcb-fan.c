/*
 * atmel-tcb-fan.c - Hwmon driver for measuring fan tachometer signals
 *		     with an Atmel Timer/Counter block.
 *
 * Copyright (C) 2018 Dennis Lambe Jr.
 *
 * Author: Dennis Lambe Jr. <dennis@profirmserv.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA	 02111-1307	 USA
 */

#include <linux/atmel_tc.h>
#include <linux/clk.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#define NCHAN_MAX 3

#define TC_REG(tc, ch, reg) ((tc)->regs + ATMEL_TC_REG(ch, reg))

struct atmel_tcb_fan_of_info
{
	int tc_block;
	int nfans;
	u32 tc_channels[NCHAN_MAX];
	const char *labels[NCHAN_MAX];
};

enum atmel_tcb_fan_state {
	ATMEL_TCB_FAN_STATE_RESET,
	ATMEL_TCB_FAN_STATE_ONE_RA,
	ATMEL_TCB_FAN_STATE_RA_VALID,
};

struct atmel_tcb_fan {
	struct atmel_tc *tc;
	spinlock_t lock;
	u32 tc_ch;
	const char *label;
	int pulses;
	enum atmel_tcb_fan_state state;
};

struct atmel_tcb_fan *atmel_tcb_fan_from_dev(struct device *dev, int hwmon_ch)
{
	struct atmel_tcb_fan *fans = dev_get_drvdata(dev);
	return &fans[hwmon_ch];
}

static ssize_t atmel_tcb_fan_label_show(struct device *dev,
					struct device_attribute *dev_attr,
					char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(dev_attr);
	struct atmel_tcb_fan *fan = atmel_tcb_fan_from_dev(dev, attr->index);

	return sprintf(buf, "%s\n", fan->label);
}

static umode_t atmel_tcb_fan_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int hwmon_ch)
{
	switch (type) {
	case hwmon_fan:
		switch  (attr) {
		case hwmon_fan_input:
		case hwmon_fan_min:
			return S_IRUGO;
		case hwmon_fan_pulses:
			return S_IRUGO | S_IWUSR;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int atmel_tcb_fan_read(struct device *dev,
			      enum hwmon_sensor_types type,
			      u32 attr, int hwmon_ch, long *val)
{
	struct atmel_tcb_fan *fan = atmel_tcb_fan_from_dev(dev, hwmon_ch);
	int pulses;
	enum atmel_tcb_fan_state state;
	unsigned long flags;
	u32 ra;

	spin_lock_irqsave(&fan->lock, flags);
	pulses = fan->pulses;
	state = fan->state;
	spin_unlock_irqrestore(&fan->lock, flags);

	if (type == hwmon_fan) {
		switch (attr) {
		case hwmon_fan_input:
			ra = __raw_readl(TC_REG(fan->tc, fan->tc_ch, RA));

			if (state == ATMEL_TCB_FAN_STATE_RA_VALID && ra)
				*val = 32767 * 60 / ra / pulses;
			else
				*val = 0;
			return 0;
		case hwmon_fan_pulses:
			*val = pulses;
			return 0;
		case hwmon_fan_min:
			*val = 60 / pulses;
			return 0;
		}
	}

	return -EINVAL;
};

static int atmel_tcb_fan_write(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int hwmon_ch, long val)
{
	struct atmel_tcb_fan *fan = atmel_tcb_fan_from_dev(dev, hwmon_ch);

	if (type == hwmon_fan && attr == hwmon_fan_pulses) {
		if (val <= 0)
			return -EINVAL;

		spin_lock(&fan->lock);
		fan->pulses = val;
		spin_unlock(&fan->lock);

		return 0;
	}

	return -EINVAL;
}

static const struct hwmon_ops atmel_tcb_fan_hwmon_ops = {
	.is_visible = atmel_tcb_fan_is_visible,
	.read = atmel_tcb_fan_read,
	.write = atmel_tcb_fan_write,
};

static irqreturn_t tcb_fan_irq(int irq, void *dev_id)
{
	struct atmel_tcb_fan *fan = dev_id;
	struct atmel_tc *tc = fan->tc;
	u32 tc_ch = fan->tc_ch;
	u32 sr = __raw_readl(TC_REG(tc, tc_ch, SR));

	switch (fan->state) {
	case ATMEL_TCB_FAN_STATE_RESET:
		if (sr & ATMEL_TC_ETRGS) {
			__raw_writel(ATMEL_TC_CPCS, TC_REG(tc, tc_ch, IER));
			fan->state = ATMEL_TCB_FAN_STATE_ONE_RA;
			return IRQ_HANDLED;
		}
		break;
	case ATMEL_TCB_FAN_STATE_ONE_RA:
		if (sr & ATMEL_TC_ETRGS) {
			__raw_writel(ATMEL_TC_ETRGS, TC_REG(tc, tc_ch, IDR));
			fan->state = ATMEL_TCB_FAN_STATE_RA_VALID;
			return IRQ_HANDLED;
		}
		/* fall through */
	case ATMEL_TCB_FAN_STATE_RA_VALID:
		if (sr & ATMEL_TC_CPCS) {
			__raw_writel(ATMEL_TC_ETRGS, TC_REG(tc, tc_ch, IER));
			fan->state = ATMEL_TCB_FAN_STATE_RESET;
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

static int atmel_tcb_fan_get_tcclks(int divisor)
{
	int tcclks;

	for (tcclks = 0; tcclks < ARRAY_SIZE(atmel_tc_divisors); tcclks++) {
		if (atmel_tc_divisors[tcclks] == divisor)
			break;
	}
	if (tcclks > ARRAY_SIZE(atmel_tc_divisors))
		return -ENODEV;

	return tcclks;
}

static int atmel_tcb_fan_parse_of(struct platform_device *pdev,
				  struct atmel_tcb_fan_of_info *info)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int err;

	err = of_property_read_u32(np, "tc-block", &info->tc_block);
	if (err < 0) {
		dev_err(dev, "failed to get Timer Counter Block number from device tree\n");
		return err;
	}

	info->nfans = of_property_read_variable_u32_array(
		np, "channels", info->tc_channels, 1, NCHAN_MAX);
	if (info->nfans < 0) {
		dev_err(dev, "failed to get channels from device tree\n");
		return info->nfans;
	}

	err = of_property_read_string_array(
		np, "channel-labels", info->labels, info->nfans);
	if (err < 0) {
		dev_err(dev, "failed to read channel-labels from device tree\n");
		return err;
	}

	return 0;
}

static struct hwmon_chip_info *ateml_tcb_fan_alloc_hwmon_chip_info(
	struct device *dev, int nfans)
{
	u32 *channel_config;
	struct hwmon_channel_info *channel_info;
	const struct hwmon_channel_info **channels_info;
	struct hwmon_chip_info *chip_info;
	int i;

	channel_config = devm_kmalloc_array(dev, nfans + 1, sizeof(u32), GFP_KERNEL);
	if (!channel_config)
		return NULL;
	channel_info = devm_kzalloc(dev, sizeof(*channel_info), GFP_KERNEL);
	if (!channel_info)
		return NULL;
	channels_info = devm_kmalloc_array(dev, 2, sizeof(*channels_info), GFP_KERNEL);
	if (!channels_info)
		return NULL;
	chip_info = devm_kzalloc(dev, sizeof(*chip_info), GFP_KERNEL);
	if (!chip_info)
		return NULL;

	for (i = 0; i < nfans; i++) {
		channel_config[i] = HWMON_F_INPUT | HWMON_F_MIN | HWMON_F_PULSES;
	}
	channel_config[i] = 0;
	channel_info->type = hwmon_fan;
	channel_info->config = channel_config;
	channels_info[0] = channel_info;
	channels_info[1] = NULL;
	chip_info->ops = &atmel_tcb_fan_hwmon_ops;
	chip_info->info = channels_info;

	return chip_info;
}

static const struct attribute_group **atmel_tcb_fan_alloc_hwmon_attr_groups(
	struct device *dev, int nfans)
{
	struct sensor_device_attribute *label_attrs;
	struct sensor_device_attribute *label_attr;
	struct attribute **attrs;
	struct attribute_group *attr_group;
	const struct attribute_group **attr_groups;
	int i;

	label_attrs = devm_kmalloc_array(dev, nfans, sizeof(*label_attrs), GFP_KERNEL);
	if (!label_attrs)
		return NULL;
	attrs = devm_kmalloc_array(dev, nfans + 1, sizeof(*attrs), GFP_KERNEL);
	if (!attrs)
		return NULL;
	attr_group = devm_kzalloc(dev, sizeof(*attr_group), GFP_KERNEL);
	if (!attr_group)
		return NULL;
	attr_groups = devm_kmalloc_array(dev, 2, sizeof(*attr_groups), GFP_KERNEL);
	if (!attr_groups)
		return NULL;

	for (i = 0; i < nfans; i++) {
		label_attr = &label_attrs[i];
		sysfs_attr_init(label_attrs->dev_attr.attr);
		label_attr->dev_attr.attr.name =
			devm_kasprintf(dev, GFP_KERNEL, "fan%d_label", i + 1);
		if (!label_attr->dev_attr.attr.name)
			return NULL;
		label_attr->dev_attr.attr.mode = S_IRUGO;
		label_attr->dev_attr.show = atmel_tcb_fan_label_show;
		label_attr->index = i;
		attrs[i] = &label_attr->dev_attr.attr;
	}
	attrs[i] = NULL;
	attr_group->attrs = attrs;
	attr_groups[0] = attr_group;
	attr_groups[1] = NULL;

	return attr_groups;
}

static void atmel_tcb_fan_free_tc(void *data)
{
	struct atmel_tc *tc = data;
	atmel_tc_free(tc);
}

static int atmel_tcb_fan_init(struct device *dev, struct atmel_tcb_fan *fan,
			      const char *label, struct atmel_tc *tc,
			      u32 tc_ch, int tcclks_32khz)
{
	int err;

	spin_lock_init(&fan->lock);
	fan->tc = tc;
	fan->tc_ch = tc_ch;
	fan->label = label;
	fan->pulses = 2;
	fan->state = ATMEL_TCB_FAN_STATE_RESET;

	err = devm_request_irq(dev, tc->irq[tc_ch], tcb_fan_irq,
			       IRQF_TIMER | IRQF_SHARED,
			       fan->label, fan);
	if (err) {
		dev_err(dev, "Failed request_irq for TC channel %d\n", tc_ch);
		return err;
	}

	err = clk_prepare_enable(tc->clk[tc_ch]);
	if (err) {
		dev_err(dev, "can't enable T%d clk\n", tc_ch);
		return err;
	}

	__raw_writel(tcclks_32khz
		     | ATMEL_TC_ETRGEDG_RISING
		     | ATMEL_TC_ABETRG
		     | ATMEL_TC_CPCTRG
		     | ATMEL_TC_LDRA_RISING,
		     TC_REG(tc, tc_ch, CMR));
	__raw_writel(32767, TC_REG(tc, tc_ch, RC));
	__raw_writel(0xff, TC_REG(tc, tc_ch, IDR));
	__raw_readl(TC_REG(tc, tc_ch, SR));
	__raw_writel(ATMEL_TC_ETRGS, TC_REG(tc, tc_ch, IER));
	__raw_writel(ATMEL_TC_CLKEN | ATMEL_TC_SWTRG, TC_REG(tc, tc_ch, CCR));

	return 0;
}

static int atmel_tcb_fan_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	struct atmel_tcb_fan *fans;
	struct atmel_tc *tc;
	struct atmel_tcb_fan_of_info info;
	int tcclks_32khz;

	const struct attribute_group **attr_groups;
	struct hwmon_chip_info *chip_info;
	struct device *hwmon_dev;

	int err;
	int i;

	tcclks_32khz = atmel_tcb_fan_get_tcclks(0);
	if (tcclks_32khz < 0) {
		dev_err(dev, "32 kHz clock divider not found\n");
		return tcclks_32khz;
	}

	err = atmel_tcb_fan_parse_of(pdev, &info);
	if (err < 0)
		return err;

	chip_info = ateml_tcb_fan_alloc_hwmon_chip_info(dev, info.nfans);
	if (!chip_info) {
		dev_err(dev, "failed to allocate memory for hwmon chip info");
		return -ENOMEM;
	}

	attr_groups = atmel_tcb_fan_alloc_hwmon_attr_groups(dev, info.nfans);
	if (!attr_groups) {
		dev_err(dev, "failed to allocate memory for attribute groups");
		return -ENOMEM;
	}

	fans = devm_kmalloc_array(dev, info.nfans, sizeof(*fans), GFP_KERNEL);
	if (!fans) {
		dev_err(dev, "failed to allcoate channel memory\n");
		return -ENOMEM;
	}

	tc = atmel_tc_alloc(info.tc_block);
	if (!tc) {
		dev_err(dev, "failed to allocate Timer Counter Block\n");
		return -ENOMEM;
	}
	devm_add_action(dev, atmel_tcb_fan_free_tc, tc);

	err = clk_prepare_enable(tc->slow_clk);
	if (err) {
		dev_err(dev, "can't enable slow clk\n");
		return err;
	}

	for (i = 0; i < info.nfans; i++) {
		err = atmel_tcb_fan_init(dev, &fans[i], info.labels[i],
					 tc, info.tc_channels[i],
					 tcclks_32khz);
		if (err < 0)
			return err;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(
		dev, "atmel_tcb_fan", fans, chip_info, attr_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "Atmel TCB fan initialized\n");

	return 0;
}

static const struct of_device_id of_atmel_tcb_fan_match[] = {
	{ .compatible = "atmel,tcb-hwmon-fan", },
	{},
};
MODULE_DEVICE_TABLE(of, of_atmel_tcb_fan_match);

static struct platform_driver atmel_tcb_fan_driver = {
	.probe = atmel_tcb_fan_probe,
	.driver = {
		.name = "atmel-tcb-fan",
		.of_match_table = of_match_ptr(of_atmel_tcb_fan_match),
	},
};

module_platform_driver(atmel_tcb_fan_driver);

MODULE_AUTHOR("Dennis Lambe Jr. <dennis@profirmserv.com>");
MODULE_ALIAS("platform:atmel-tcb-fan");
MODULE_DESCRIPTION("Atmel TC Block fan tachometer driver");
MODULE_LICENSE("GPL");
