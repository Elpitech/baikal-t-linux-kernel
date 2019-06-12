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
	kexec_args[0] = 0;
	kexec_args[1] = 0;
	kexec_args[2] = 0;
	kexec_args[3] = 0;

	pr_info("baikal-kexec - baikal_kexec_prepare: kimage %p\n", kimage);

	be_kexec_set_dtb(kimage);

	return 0;
}

void baikal_kexec_shutdown(void)
{
	pr_info("baikal-kexec: baikal_kexec_shutdown\n");

	__flush_cache_all();

	/* Shutdown CPU core1 */
	baikal_kexec_core_down();
}
