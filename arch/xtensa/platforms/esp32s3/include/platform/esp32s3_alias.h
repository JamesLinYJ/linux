/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _PLATFORM_ESP32S3_ALIAS_H
#define _PLATFORM_ESP32S3_ALIAS_H

#include <linux/types.h>

/* Virtual apertures sharing MMU entries, not installed RAM sizes. */
#define ESP32S3_EXTMEM_DATA_BASE	0x3c000000UL
#define ESP32S3_EXTMEM_EXEC_BASE	0x42000000UL
#define ESP32S3_EXTMEM_WINDOW_SIZE	0x02000000UL
#define ESP32S3_FDPIC_MAX_LOAD_SEGS	32

static inline bool esp32s3_alias_data_range(unsigned long start,
					    unsigned long end)
{
	return start >= ESP32S3_EXTMEM_DATA_BASE && end >= start &&
	       end <= ESP32S3_EXTMEM_DATA_BASE + ESP32S3_EXTMEM_WINDOW_SIZE;
}

static inline unsigned long esp32s3_alias_exec(unsigned long address)
{
	if (address - ESP32S3_EXTMEM_DATA_BASE < ESP32S3_EXTMEM_WINDOW_SIZE)
		return address - ESP32S3_EXTMEM_DATA_BASE + ESP32S3_EXTMEM_EXEC_BASE;
	return address;
}

int esp32s3_alias_cache_sync(unsigned long data_start, unsigned long exec_start,
			     unsigned long length);

#endif
