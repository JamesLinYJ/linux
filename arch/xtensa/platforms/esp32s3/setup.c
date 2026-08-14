// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal native ESP32-S3 platform boundary.
 *
 * Hardware controllers are added as separate, reviewable subsystem patches.
 * Keeping this file free of bootloader services makes the ownership boundary
 * explicit: after the handoff, Linux is the only resident runtime.
 */

#include <linux/init.h>
#include <linux/printk.h>

#include <asm/platform.h>

void __init platform_setup(char **command_line)
{
	(void)command_line;
	pr_info("ESP32-S3: native Linux platform (experimental)\n");
}
