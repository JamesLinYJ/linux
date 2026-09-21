// SPDX-License-Identifier: GPL-2.0-only
/*
 * External MSPI cache boundary. ROM addresses/signatures were checked against
 * Espressif ESP-IDF v5.5.3 (2c211b236707889e8400c4dc5644dd5c4ee071e0),
 * components/esp_rom/esp32s3/{ld/esp32s3.rom.ld,include/esp32s3/rom/cache.h}.
 * Only hardware interface facts are used; no ESP-IDF implementation is linked.
 */
#include <linux/align.h>
#include <linux/errno.h>
#include <linux/irqflags.h>
#include <linux/overflow.h>

#include <platform/esp32s3_alias.h>

#define ESP32S3_ROM_CACHE_GET_DCACHE_LINE_SIZE	0x40001608UL
#define ESP32S3_ROM_CACHE_INVALIDATE_ADDR		0x400016b0UL
#define ESP32S3_ROM_CACHE_WRITEBACK_ADDR		0x400016c8UL
#define ESP32S3_ROM_CACHE_SUSPEND_DCACHE_AUTOLOAD	0x40001734UL
#define ESP32S3_ROM_CACHE_RESUME_DCACHE_AUTOLOAD	0x40001740UL

int esp32s3_alias_cache_sync(unsigned long data_start, unsigned long exec_start,
			     unsigned long length)
{
	u32 (*get_line_size)(void) = (void *)ESP32S3_ROM_CACHE_GET_DCACHE_LINE_SIZE;
	int (*writeback)(u32, u32) = (void *)ESP32S3_ROM_CACHE_WRITEBACK_ADDR;
	int (*invalidate)(u32, u32) = (void *)ESP32S3_ROM_CACHE_INVALIDATE_ADDR;
	u32 (*suspend_autoload)(void) = (void *)ESP32S3_ROM_CACHE_SUSPEND_DCACHE_AUTOLOAD;
	void (*resume_autoload)(u32) = (void *)ESP32S3_ROM_CACHE_RESUME_DCACHE_AUTOLOAD;
	unsigned long data_end, flags, offset;
	u32 autoload_state, line_size;
	int ret;

	if (check_add_overflow(data_start, length, &data_end) ||
	    !esp32s3_alias_data_range(data_start, data_end) ||
	    exec_start != esp32s3_alias_exec(data_start))
		return -EINVAL;
	if (!length)
		return 0;

	line_size = get_line_size();
	if (line_size != 16 && line_size != 32 && line_size != 64)
		return -EIO;
	offset = data_start & (line_size - 1);
	data_start -= offset;
	exec_start -= offset;
	/* The aperture check bounds this sum well below ULONG_MAX. */
	length = ALIGN(length + offset, line_size);

	/* Whole lines avoid the ROM partial-line writeback erratum. */
	local_irq_save(flags);
	autoload_state = suspend_autoload();
	ret = writeback(data_start, length);
	resume_autoload(autoload_state);
	if (!ret)
		ret = invalidate(exec_start, length);
	local_irq_restore(flags);

	return ret ? -EIO : 0;
}
