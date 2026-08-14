// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal native ESP32-S3 platform boundary.
 *
 * Hardware controllers are added as separate, reviewable subsystem patches.
 * Keeping this file free of bootloader services makes the ownership boundary
 * explicit: after the handoff, Linux is the only resident runtime.
 */

#include <linux/init.h>
#include <linux/io.h>
#include <linux/printk.h>
#include <linux/reboot.h>

#include <asm/platform.h>

#define ESP32S3_RTC_CNTL_OPTIONS0	0x60008000
#define ESP32S3_RTC_CNTL_SW_SYS_RST	BIT(31)

static int esp32s3_restart(struct sys_off_data *unused)
{
	void __iomem *options0 =
		(void __iomem *)(unsigned long)ESP32S3_RTC_CNTL_OPTIONS0;

	writel(ESP32S3_RTC_CNTL_SW_SYS_RST, options0);
	pr_emerg("ESP32-S3: software system reset failed\n");

	return NOTIFY_DONE;
}

void __init platform_setup(char **command_line)
{
	(void)command_line;
	pr_info("ESP32-S3: native Linux platform (experimental)\n");

	register_sys_off_handler(SYS_OFF_MODE_RESTART,
				 SYS_OFF_PRIO_PLATFORM,
				 esp32s3_restart, NULL);
}
