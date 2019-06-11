/*
 * Baikal-T SOC platform support code. Kexec support functions.
 *
 * Copyright (C) 2014-2017 Baikal Electronics JSC
 * 
 * Author:
 *   Alexander Sazonov <Alexander.Sazonov@baikalelectronics.ru>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/compiler.h>
#include <linux/kexec.h>
#include <linux/libfdt.h>
#include <linux/cpumask.h>

#include <asm/cacheflush.h>
#include <asm/page.h>
#include <asm/io.h>

#include <asm/uaccess.h>
#include <asm/bootinfo.h>
#include <asm/smp-cps.h>
#include <asm/mips-cpc.h>
#include <asm/machine_kexec.h>

/*
 * Boot parameter processing
 */

extern unsigned long fw_arg0, fw_arg1, fw_arg2, fw_arg3;

/*
static void be_kexec_print_args(void)
{
	unsigned long argc = (int)kexec_args[0];
	int i;

	pr_info("kexec_args[0] (argc): %lu\n", argc);
	pr_info("kexec_args[1] (argv): %p\n", (void *)kexec_args[1]);
	pr_info("kexec_args[2] (env ): %p\n", (void *)kexec_args[2]);
	pr_info("kexec_args[3] (dtb ): %p\n", (void *)kexec_args[3]);

	for (i = 0; i < argc; i++) {
		pr_info("kexec_argv[%d] = %p, %s\n",
				i, kexec_argv[i], kexec_argv[i]);
	}
}
*/

static int be_kexec_set_dtb(struct kimage *image)
{
	struct kexec_segment *seg;
	int i;

	pr_info("baikal-kexec - be_kexec_set_dtb: image %p\n", image);

	for (i = 0; i < image->nr_segments; i++) {
		seg = &image->segment[i];
		if (!seg || !seg->buf)
			continue;

		if (fdt_check_header((void *)seg->buf) == 0) {
			pr_debug("be_kexec_set_dtb: fdt_check_header: magic=0x%08x, size=%u, version=0x%08x\n",
				fdt_magic(seg->buf), fdt_totalsize(seg->buf), fdt_version(seg->buf));
			pr_info("be_kexec_set_dtb: seg[%d]: [buf %zu @ %p], [mem %zu @ 0x%lx]\n",
				i, seg->bufsz, seg->buf, seg->memsz, seg->mem);

			kexec_args[3] = seg->mem; /* addr of dtb */
			return 0;
		}
	}

	return -EINVAL;
}

static void be_kexec_init_argv(struct kimage *image)
{
	void __user *buf = NULL;
	size_t size = KEXEC_COMMAND_LINE_SIZE;
	size_t bufsz = 0;
	struct kexec_segment *seg;
	int i;

	pr_info("baikal-kexec - be_kexec_init_argv: image %p\n", image);

	for (i = 0; i < image->nr_segments; i++) {
		seg = &image->segment[i];
		if (seg->bufsz < 6)
			continue;

		if (strncmp((char *) seg->buf, "kexec\x20", 6))
			continue;

		/* don't copy "kexec" */
		buf = seg->buf + 6;
		bufsz = seg->bufsz - 6;
		break;
	}

	if (!buf)
		return;

	size = min(size, bufsz);
	if (size < bufsz)
		pr_warn("baikal-kexec - be_kexec_init_argv: command line truncated to %zd bytes\n", size);

	/* Copy to kernel space */
	copy_from_user(kexec_argv_buf, buf, size);
	kexec_argv_buf[size - 1] = 0;
}

static void be_kexec_parse_argv(struct kimage *image)
{
	char *reboot_code_buffer;
	int reloc_delta;
	char *ptr = kexec_argv_buf;
	int argc = 0;
	int i;

	pr_info("baikal-kexec - be_kexec_parse_argv: image %p\n", image);

	/* convert command line string to array of parameters (as bootloader does) */
	while (ptr && *ptr && (argc < KEXEC_MAX_ARGC)) {
		if (*ptr == ' ') {
			*ptr++ = '\0';
			continue;
		}

		kexec_argv[argc++] = ptr;
		ptr = strchr(ptr, ' ');
	}

	if (!argc) {
		return;
	}

	pr_info("   # argc    = %d\n", argc);
	for (i = 0; i < argc; i++) {
		pr_info("   # argv[%d] = %s\n", i, kexec_argv[i]);
	}

	kexec_args[0] = argc;						/* # of cmdline params */
	kexec_args[1] = (unsigned long)kexec_argv;	/* cmdline */
	kexec_args[2] = 0;							/* addr of env */
	kexec_args[3] = 0;							/* addr of dtb */

	reboot_code_buffer = page_address(image->control_code_page);
	reloc_delta = reboot_code_buffer - (char *)kexec_relocate_new_kernel;

	kexec_args[1] += reloc_delta;
	for (i = 0; i < argc; i++)
		kexec_argv[i] += reloc_delta;
}

/*
 * CPU core power-down functions
 */

static volatile unsigned int down_timer;
#define DOWN_TMR_MAX 100000

/* Statically configured core = 1 since Baikal-T1 is 2-core CPU */
#define BE_PWRDOWN_CORE 1

static int baikal_kexec_core_down(void)
{
	int ret = -1;
	u32 stat = 0;
	u32 sec_state = 0;
	u32 access = 0;

	local_irq_disable();

	if (!cpumask_test_cpu(BE_PWRDOWN_CORE, cpu_online_mask)) {
		pr_warn("baikal-kexec - baikal_kexec_core_down: core 1 not online!\n");
	}

	/* Select the appropriate core (1) */
	mips_cm_lock_other(BE_PWRDOWN_CORE, 0);

	/* Ensure its coherency is disabled */
	write_gcr_co_coherence(0);

	/* Ensure the core can access the GCRs */
	access = read_gcr_access();
	access |= 1 << (CM_GCR_ACCESS_ACCESSEN_SHF + BE_PWRDOWN_CORE);
	write_gcr_access(access);

	if (!mips_cpc_present()) {
		pr_err("baikal-kexec - baikal_kexec_core_down: !mips_cpc_present\n");
		mips_cm_unlock_other();
		local_irq_enable();
		return -1;
	}

	mips_cpc_lock_other(BE_PWRDOWN_CORE);

	stat = read_cpc_co_stat_conf();
	sec_state = (stat & CPC_Cx_STAT_CONF_SEQSTATE_MSK) >> CPC_Cx_STAT_CONF_SEQSTATE_SHF;
	pr_debug("baikal-kexec - baikal_kexec_core_down: core 1 SEQ_STATE=0x%02x\n", sec_state);

	/* Power down core 1 with CPC_Cx_CMD_PWRDOWN */
	if (sec_state != 0) {
		pr_info("baikal-kexec - baikal_kexec_core_down: power down core 1...\n");
		write_cpc_co_cmd(CPC_Cx_CMD_PWRDOWN);
		for (down_timer = 0; down_timer < DOWN_TMR_MAX; down_timer++) {
			stat = read_cpc_co_stat_conf();
			sec_state = (stat & CPC_Cx_STAT_CONF_SEQSTATE_MSK) >> CPC_Cx_STAT_CONF_SEQSTATE_SHF;
			if (sec_state == 0) {
				set_cpu_online(BE_PWRDOWN_CORE, 0);
				ret = 0;
				break;
			}
		}
		if (down_timer == DOWN_TMR_MAX) {
			pr_err("baikal-kexec - baikal_kexec_core_down: power down core 1: timeout!\n");
			ret = -1;
		}
	}

	if (cpumask_test_cpu(BE_PWRDOWN_CORE, cpu_online_mask)) {
		pr_err("baikal-kexec - baikal_kexec_core_down: core 1 still online!\n");
	}

	mips_cpc_unlock_other();

	mips_cm_unlock_other();

	local_irq_enable();
	return ret;
}


/*
 * Kexec-* interface functions
 */

/* extern void dw_pcie_kexec_prepare(void); */

int baikal_kexec_prepare(struct kimage *kimage)
{
	/*
	 * Whenever arguments passed from kexec-tools, Init the arguments as
	 * the original ones to try avoiding booting failure.
	 */

	pr_info("baikal-kexec - baikal_kexec_prepare: kimage %p\n", kimage);

	/*
	printk("   = fw_arg0 (argc): 0x%lx\n", fw_arg0);
	printk("   = fw_arg1 (argv): 0x%lx\n", fw_arg1);
	printk("   = fw_arg2 (env ): 0x%lx\n", fw_arg2);
	printk("   = fw_arg3 (fdt ): 0x%lx\n", fw_arg3);
	*/

	/* Comments from u-boot
	 * fw_arg0 contains argc value (arguments count)
	 * fw_arg1 contains argv pointer (arguments values)
	 * fw_arg2 contains pointer to bootloader enviroment variables (not used in Baikal)
	 * fw_arg3 used to point to DTB location
	 * like this:
	 * void *fdt = IS_BUILTIN(CONFIG_BUILTIN_DTB) ? __dtb_start : phys_to_virt(fw_arg3);
	 * ( from mips/baikal/baikal-of.c: device_tree_early_init(void) )
	 */
	kexec_args[0] = fw_arg0;
	kexec_args[1] = fw_arg1;
	kexec_args[2] = fw_arg2;
	kexec_args[3] = fw_arg3;

	be_kexec_init_argv(kimage);
	be_kexec_parse_argv(kimage);
	be_kexec_set_dtb(kimage);

	return 0;
}

void baikal_kexec_shutdown(void)
{
	pr_info("baikal-kexec: baikal_kexec_shutdown\n");

	/* Reset PCIe */
	/* dw_pcie_kexec_prepare(); */

	__flush_cache_all();

	/* Shutdown CPU core1 */
	baikal_kexec_core_down();
}
