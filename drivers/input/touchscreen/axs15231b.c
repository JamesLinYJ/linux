// SPDX-License-Identifier: GPL-2.0-only
/*
 * AXS Technology AXS15231B in-cell touchscreen controller
 *
 * The wire protocol is documented by the AXS15231B specification and the
 * board vendor's ESP-IDF component. This driver is an independent Linux input
 * implementation and does not depend on an ESP-IDF runtime.
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include "axs15231b.h"

static const u8 axs15231b_read_command[] = {
	0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00,
	0x00, AXS15231B_REPORT_LEN, 0x00, 0x00, 0x00,
};

struct axs15231b_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
};

static int axs15231b_read_report(struct axs15231b_data *data,
				 struct axs15231b_point *point)
{
	u8 report[AXS15231B_REPORT_LEN];
	int ret;

	ret = i2c_master_send(data->client, axs15231b_read_command,
			      sizeof(axs15231b_read_command));
	if (ret < 0)
		return ret;
	if (ret != (int)sizeof(axs15231b_read_command))
		return -EIO;

	ret = i2c_master_recv(data->client, report, sizeof(report));
	if (ret < 0)
		return ret;
	if (ret != (int)sizeof(report))
		return -EIO;

	return axs15231b_parse_report(report, sizeof(report), point);
}

static irqreturn_t axs15231b_irq_thread(int irq, void *dev_id)
{
	struct axs15231b_data *data = dev_id;
	struct device *dev = &data->client->dev;
	struct axs15231b_point point;
	int ret;

	ret = axs15231b_read_report(data, &point);
	if (ret) {
		dev_err_ratelimited(dev, "failed to read touch report: %d\n", ret);
		return IRQ_HANDLED;
	}

	if (point.active &&
	    (point.x > data->prop.max_x || point.y > data->prop.max_y)) {
		dev_warn_ratelimited(dev, "touch coordinate out of range: %u,%u\n",
				     point.x, point.y);
		return IRQ_HANDLED;
	}

	input_report_key(data->input, BTN_TOUCH, point.active);
	if (point.active)
		touchscreen_report_pos(data->input, &data->prop,
				       point.x, point.y, false);
	input_sync(data->input);

	return IRQ_HANDLED;
}

static int axs15231b_open(struct input_dev *input)
{
	struct axs15231b_data *data = input_get_drvdata(input);

	enable_irq(data->client->irq);
	return 0;
}

static void axs15231b_close(struct input_dev *input)
{
	struct axs15231b_data *data = input_get_drvdata(input);

	disable_irq(data->client->irq);
}

static int axs15231b_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct axs15231b_data *data;
	struct input_dev *input;
	int ret;

	if (!client->irq)
		return dev_err_probe(dev, -EINVAL, "interrupt is required\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks plain I2C transfers\n");

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	data->client = client;
	data->input = input;

	input->name = "AXS15231B Touchscreen";
	input->id.bustype = BUS_I2C;
	input->dev.parent = dev;
	input->open = axs15231b_open;
	input->close = axs15231b_close;
	input_set_drvdata(input, data);

	input_set_capability(input, EV_KEY, BTN_TOUCH);
	input_set_abs_params(input, ABS_X, 0, 0, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 0, 0, 0);
	touchscreen_parse_properties(input, false, &data->prop);
	if (!input_abs_get_max(input, ABS_X) ||
	    !input_abs_get_max(input, ABS_Y))
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen dimensions are required\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					axs15231b_irq_thread, IRQF_ONESHOT,
					dev_name(dev), data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request interrupt\n");

	disable_irq(client->irq);

	i2c_set_clientdata(client, data);

	ret = input_register_device(input);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register input device\n");

	return 0;
}

static int axs15231b_suspend(struct device *dev)
{
	struct axs15231b_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		axs15231b_close(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static int axs15231b_resume(struct device *dev)
{
	struct axs15231b_data *data = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&data->input->mutex);
	if (input_device_enabled(data->input))
		axs15231b_open(data->input);
	mutex_unlock(&data->input->mutex);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(axs15231b_pm_ops,
				axs15231b_suspend, axs15231b_resume);

static const struct of_device_id axs15231b_of_match[] = {
	{ .compatible = "axs,axs15231b" },
	{ }
};
MODULE_DEVICE_TABLE(of, axs15231b_of_match);

static struct i2c_driver axs15231b_driver = {
	.driver = {
		.name = "axs15231b",
		.of_match_table = axs15231b_of_match,
		.pm = pm_sleep_ptr(&axs15231b_pm_ops),
	},
	.probe = axs15231b_probe,
};
module_i2c_driver(axs15231b_driver);

MODULE_AUTHOR("ESP32-S3 Linux project");
MODULE_DESCRIPTION("AXS15231B touchscreen driver");
MODULE_LICENSE("GPL");
