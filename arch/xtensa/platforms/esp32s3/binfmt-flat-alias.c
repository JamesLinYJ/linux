// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S3 Harvard-alias boundary for the generic bFLT loader.
 *
 * Keep fs/binfmt_flat.c unchanged and compile that implementation here.  The
 * ESP32-S3 PSRAM is visible through a data-bus alias at 0x3c000000 and an
 * instruction-bus alias at 0x42000000.  A RAM-loaded bFLT image therefore
 * needs only three platform-specific translations: relocated code pointers,
 * the initial PC, and instruction-cache invalidation.
 */

#include <linux/align.h>
#include <linux/mm_types.h>
#include <linux/overflow.h>
#include <linux/printk.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/types.h>

#include <asm/cacheflush.h>
#include <asm/flat.h>
#include <asm/processor.h>

#define ESP32S3_PSRAM_DATA_BASE	0x3c000000UL
#define ESP32S3_PSRAM_EXEC_BASE	0x42000000UL
#define ESP32S3_PSRAM_SIZE	0x00800000UL
/* ESP32-S3 ROM cache ABI, from the official esp32s3.rom.ld. */
#define ESP32S3_ROM_CACHE_GET_DCACHE_LINE_SIZE	0x40001608UL
#define ESP32S3_ROM_CACHE_INVALIDATE_ADDR	0x400016b0UL
#define ESP32S3_ROM_CACHE_WRITEBACK_ADDR		0x400016c8UL
#define ESP32S3_ROM_CACHE_SUSPEND_DCACHE_AUTOLOAD	0x40001734UL
#define ESP32S3_ROM_CACHE_RESUME_DCACHE_AUTOLOAD	0x40001740UL

typedef int (*esp32s3_rom_cache_range_fn)(u32 address, u32 length);
typedef u32 (*esp32s3_rom_cache_get_line_size_fn)(void);
typedef u32 (*esp32s3_rom_cache_suspend_fn)(void);
typedef void (*esp32s3_rom_cache_resume_fn)(u32 state);

static const char esp32s3_flat_alias_marker[] __used =
	"ESP32-S3 bFLT Harvard alias boundary";

struct esp32s3_flat_cache_range {
	unsigned long data_start;
	unsigned long exec_start;
	unsigned long length;
};

static bool esp32s3_flat_data_range(unsigned long start, unsigned long end)
{
	return start >= ESP32S3_PSRAM_DATA_BASE && end >= start &&
	       end <= ESP32S3_PSRAM_DATA_BASE + ESP32S3_PSRAM_SIZE;
}

static unsigned long esp32s3_flat_exec_alias(unsigned long address)
{
	if (address < ESP32S3_PSRAM_DATA_BASE ||
	    address >= ESP32S3_PSRAM_DATA_BASE + ESP32S3_PSRAM_SIZE)
		return address;

	return ESP32S3_PSRAM_EXEC_BASE +
	       (address - ESP32S3_PSRAM_DATA_BASE);
}

static int esp32s3_flat_put_addr_at_rp(u32 __user *rp, u32 address, u32 rel)
{
	struct mm_struct *mm = current->mm;

	/*
	 * Generic bFLT relocation has already classified and range-checked the
	 * target.  Preserve data pointers, but convert pointers into the loaded
	 * text interval to the executable alias before storing them.
	 */
	if (mm && address >= mm->start_code && address < mm->end_code)
		address = esp32s3_flat_exec_alias(address);

	return flat_put_addr_at_rp(rp, address, rel);
}

static void esp32s3_flat_start_thread(struct pt_regs *regs,
				      unsigned long pc, unsigned long sp)
{
	start_thread(regs, esp32s3_flat_exec_alias(pc), sp);
}

static void esp32s3_flat_plat_init(struct pt_regs *regs)
{
	/*
	 * The Xtensa C runtime treats a2 as the dynamic linker's rtld_fini
	 * callback.  A static bFLT image has no dynamic linker, so carrying a
	 * register value over from the previous kernel context would turn that
	 * value into a userspace function pointer.
	 */
	regs->areg[2] = 0;
}

static void esp32s3_flat_cache_sync(void *argument)
{
	const struct esp32s3_flat_cache_range *range = argument;
	esp32s3_rom_cache_get_line_size_fn get_line_size;
	esp32s3_rom_cache_range_fn writeback;
	esp32s3_rom_cache_range_fn invalidate;
	esp32s3_rom_cache_suspend_fn suspend_autoload;
	esp32s3_rom_cache_resume_fn resume_autoload;
	unsigned long data_start, exec_start, length, offset;
	u32 autoload_state, line_size;
	int error;

	/*
	 * ESP32-S3 reports no Xtensa-local cache in XCHAL_* because PSRAM is
	 * behind the SoC MSPI cache.  The generic Xtensa helpers consequently
	 * compile to no-ops; use the official ROM cache ABI for this external
	 * cache instead.  Aligning the complete interval also avoids the ROM
	 * writeback erratum for partial cache lines.
	 */
	get_line_size = (esp32s3_rom_cache_get_line_size_fn)
		ESP32S3_ROM_CACHE_GET_DCACHE_LINE_SIZE;
	line_size = get_line_size();
	if (line_size != 16 && line_size != 32 && line_size != 64) {
		pr_err_ratelimited("invalid external DCache line size: %u\n",
				   line_size);
		return;
	}

	offset = range->data_start & (line_size - 1);
	data_start = range->data_start - offset;
	exec_start = range->exec_start - offset;
	if (check_add_overflow(range->length, offset, &length)) {
		pr_err_ratelimited("bFLT cache range overflow\n");
		return;
	}
	length = ALIGN(length, line_size);

	writeback = (esp32s3_rom_cache_range_fn)
		ESP32S3_ROM_CACHE_WRITEBACK_ADDR;
	invalidate = (esp32s3_rom_cache_range_fn)
		ESP32S3_ROM_CACHE_INVALIDATE_ADDR;
	suspend_autoload = (esp32s3_rom_cache_suspend_fn)
		ESP32S3_ROM_CACHE_SUSPEND_DCACHE_AUTOLOAD;
	resume_autoload = (esp32s3_rom_cache_resume_fn)
		ESP32S3_ROM_CACHE_RESUME_DCACHE_AUTOLOAD;

	autoload_state = suspend_autoload();
	error = writeback(data_start, length);
	resume_autoload(autoload_state);
	if (!error)
		error = invalidate(exec_start, length);
	if (error)
		pr_err_ratelimited("bFLT external cache sync failed: %d\n",
				   error);
}

static void esp32s3_flat_flush_fallback(unsigned long start,
					unsigned long end)
{
	flush_icache_user_range(start, end);
}

static void esp32s3_flat_flush_icache_user_range(unsigned long start,
						 unsigned long end)
{
	struct esp32s3_flat_cache_range range;

	if (!esp32s3_flat_data_range(start, end)) {
		esp32s3_flat_flush_fallback(start, end);
		return;
	}

	range.data_start = start;
	range.exec_start = esp32s3_flat_exec_alias(start);
	range.length = end - start;

#ifdef CONFIG_SMP
	on_each_cpu(esp32s3_flat_cache_sync, &range, 1);
#else
	esp32s3_flat_cache_sync(&range);
#endif
}

/* Redirect only the architecture boundary calls made by the generic loader. */
#define flat_put_addr_at_rp(rp, address, rel) \
	esp32s3_flat_put_addr_at_rp((rp), (address), (rel))

#undef start_thread
#define start_thread(regs, pc, sp) \
	esp32s3_flat_start_thread((regs), (pc), (sp))

#undef flush_icache_user_range
#define flush_icache_user_range(start, end) \
	esp32s3_flat_flush_icache_user_range((start), (end))

#define FLAT_PLAT_INIT(regs) esp32s3_flat_plat_init(regs)

/* Reuse the pinned mainline loader verbatim; no forked implementation copy. */
#undef pr_fmt
#include "../../../../fs/binfmt_flat.c"

#undef FLAT_PLAT_INIT
#undef flush_icache_user_range
#undef start_thread
#undef flat_put_addr_at_rp
