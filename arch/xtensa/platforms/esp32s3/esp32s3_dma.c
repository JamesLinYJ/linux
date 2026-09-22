// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S3 external MSPI cache ownership for the Linux streaming DMA API.
 * ROM interface facts: Espressif ESP-IDF v5.5.3, esp32s3.rom.ld/cache.h.
 * The core's XCHAL_DCACHE_SIZE is zero; it does not describe this cache.
 */
#include <linux/align.h>
#include <linux/dma-map-ops.h>
#include <linux/irqflags.h>
#include <linux/overflow.h>
#include <asm/io.h>
#include <platform/esp32s3_alias.h>

#ifndef CONFIG_MMU
#define ROM_INVALIDATE_ADDR	0x400016b0UL
#define ROM_WRITEBACK_ADDR	0x400016c8UL
#define ROM_SUSPEND_AUTOLOAD	0x40001734UL
#define ROM_RESUME_AUTOLOAD	0x40001740UL

static void esp32s3_dma_cache_op(phys_addr_t paddr, size_t size,
				 bool writeback, bool invalidate)
{
	int (*wb)(u32, u32) = (void *)ROM_WRITEBACK_ADDR;
	int (*inv)(u32, u32) = (void *)ROM_INVALIDATE_ADDR;
	u32 (*suspend)(void) = (void *)ROM_SUSPEND_AUTOLOAD;
	void (*resume)(u32) = (void *)ROM_RESUME_AUTOLOAD;
	unsigned long start = (unsigned long)phys_to_virt(paddr);
	unsigned long end, flags;
	u32 state;
	int ret = 0;

	if (!size || check_add_overflow(start, size, &end))
		return;
	/* Internal SRAM, including the coherent descriptor pool, is uncached. */
	if (!esp32s3_alias_data_range(start, end))
		return;
	/* DMA allocations are isolated at the largest supported cache line. */
	start = ALIGN_DOWN(start, ARCH_DMA_MINALIGN);
	end = ALIGN(end, ARCH_DMA_MINALIGN);
	local_irq_save(flags);
	state = suspend();
	if (writeback)
		ret = wb(start, end - start);
	if (!ret && invalidate)
		ret = inv(start, end - start);
	resume(state);
	local_irq_restore(flags);
	WARN_ON_ONCE(ret);
}

void arch_sync_dma_for_device(phys_addr_t paddr, size_t size,
			     enum dma_data_direction dir)
{
	/* Preserve dirty CPU bytes before granting the device write ownership. */
	esp32s3_dma_cache_op(paddr, size, true, dir != DMA_TO_DEVICE);
}

void arch_sync_dma_for_cpu(phys_addr_t paddr, size_t size,
			  enum dma_data_direction dir)
{
	if (dir != DMA_TO_DEVICE)
		esp32s3_dma_cache_op(paddr, size, false, true);
}
#endif
