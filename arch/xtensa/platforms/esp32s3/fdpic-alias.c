// SPDX-License-Identifier: GPL-2.0-only
/* ESP32-S3 Harvard address boundary for the Linux FDPIC loader. */
#include <linux/elf-fdpic.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/overflow.h>

#include <platform/esp32s3_alias.h>

/* Called for executable AND interpreter, before begin_new_exec(). */
int esp32s3_fdpic_check(struct elf_fdpic_params *params, struct file *file)
{
	const struct elfhdr *hdr = &params->hdr;
	unsigned int i, j, nloads = 0;
	loff_t file_size = i_size_read(file_inode(file));
	bool entry_valid = false;

	if (!elf_check_fdpic(hdr) || hdr->e_ident[EI_CLASS] != ELFCLASS32 ||
	    hdr->e_ident[EI_DATA] != ELFDATA2LSB || !hdr->e_entry || file_size < 0)
		return -ENOEXEC;

	for (i = 0; i < hdr->e_phnum; i++) {
		const struct elf_phdr *ph = &params->phdrs[i];
		u32 end;

		if (ph->p_type == PT_GNU_STACK && (ph->p_flags & PF_X))
			return -ENOEXEC;
		if (ph->p_type != PT_LOAD)
			continue;
		if (++nloads > ESP32S3_FDPIC_MAX_LOAD_SEGS || !ph->p_memsz ||
		    ph->p_filesz > ph->p_memsz ||
		    check_add_overflow(ph->p_vaddr, ph->p_memsz, &end) ||
		    (u64)ph->p_offset + ph->p_filesz > (u64)file_size ||
		    ((ph->p_offset ^ ph->p_vaddr) & ~PAGE_MASK) ||
		    (ph->p_align > 1 && (!is_power_of_2(ph->p_align) ||
		     ((ph->p_offset ^ ph->p_vaddr) & (ph->p_align - 1)))) ||
		    (ph->p_flags & (PF_W | PF_X)) == (PF_W | PF_X))
			return -ENOEXEC;

		/* A read-only Flash mapping cannot be used to clear BSS. */
		if (!(ph->p_flags & PF_W) && ph->p_filesz != ph->p_memsz)
			return -ENOEXEC;
		for (j = 0; j < i; j++) {
			const struct elf_phdr *prev = &params->phdrs[j];

			if (prev->p_type == PT_LOAD && ph->p_vaddr <
			    prev->p_vaddr + prev->p_memsz && prev->p_vaddr < end)
				return -ENOEXEC;
		}
		if ((ph->p_flags & PF_X) && hdr->e_entry >= ph->p_vaddr &&
		    hdr->e_entry < end)
			entry_valid = true;
	}

	return nloads && entry_valid ? 0 : -ENOEXEC;
}

/*
 * Keep VMAs at their mmap/DBus addresses for copying, unmap and accounting.
 * Translate only the kernel-owned loadmap and PC, before AT_ENTRY and the
 * user loadmap are written. No process-wide start_code/end_code guess is used.
 */
int esp32s3_fdpic_finalize_map(struct elf_fdpic_params *params,
			       struct mm_struct *mm)
{
	struct elf_fdpic_loadmap *map = params->loadmap;
	bool entry_valid = false;
	unsigned int i;
	int ret = -ENOEXEC;

	if (!mm || !map || map->version != ELF_FDPIC_LOADMAP_VERSION ||
	    !map->nsegs || map->nsegs > ESP32S3_FDPIC_MAX_LOAD_SEGS)
		return -ENOEXEC;

	mmap_read_lock(mm);
	for (i = 0; i < map->nsegs; i++) {
		struct elf_fdpic_loadseg *seg = &map->segs[i];
		unsigned long start = seg->addr, end;
		struct vm_area_struct *vma;

		if (!seg->p_memsz || check_add_overflow(start,
							(unsigned long)seg->p_memsz, &end) ||
		    !esp32s3_alias_data_range(start, end))
			goto out;
		vma = find_vma(mm, start);
		if (!vma || start < vma->vm_start || end > vma->vm_end)
			goto out;
		if (vma->vm_flags & VM_EXEC) {
			if (vma->vm_flags & VM_WRITE)
				goto out;
			ret = esp32s3_alias_cache_sync(start,
						       esp32s3_alias_exec(start),
						    seg->p_memsz);
			if (ret)
				goto out;
			if (params->entry_addr >= start && params->entry_addr < end) {
				params->entry_addr = esp32s3_alias_exec(params->entry_addr);
				entry_valid = true;
			}
			seg->addr = esp32s3_alias_exec(start);
		}
		ret = -ENOEXEC;
	}
	ret = entry_valid ? 0 : -ENOEXEC;
out:
	mmap_read_unlock(mm);
	return ret;
}
