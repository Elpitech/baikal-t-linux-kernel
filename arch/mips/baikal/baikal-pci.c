/*
 *  Baikal-T SOC platform support code.
 *
 *  Copyright (C) 2015-2017 Baikal Electronics JSC.
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 *  BAIKAL MIPS boards PCI hardware initialization.
 */
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>

#include <asm/mach-baikal/pci-baikal.h>

#define PCIE_PHY_RETRIES	1000000
#define PCIE_ERROR_VALUE	0xFFFFFFFF
#define PHY_ALL_LANES		0xF
#define PHY_LANE0		0x1

#define OK			0
#define ERROR			-1

static uint32_t dw_pcie_phy_read(uint32_t phy_addr)
{
	uint32_t reg;
	int i;

	/* Set lane0 for reading values. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_SEL_LANE, PHY_LANE0);

	/* Write the address of the PHY register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK) | BK_MGMT_CTRL_READ);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = READ_PCIE_REG(PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE) {
			/* Read data register. */
			reg = READ_PCIE_REG(PCIE_BK_MGMT_READ_DATA);
			pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__, phy_addr, reg);
			return reg;
		}
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return PCIE_ERROR_VALUE;
}

static uint32_t __maybe_unused dw_pcie_phy_write(uint32_t phy_addr, uint32_t val)
{
	uint32_t reg;
	int i;

	pr_debug("%s: phy_addr=0x%x val=0x%x\n", __func__, phy_addr, val);

	/* Set line. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_SEL_LANE, PHY_ALL_LANES);

	/* Write value to data register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_WRITE_DATA, val);

	/* Write the address of the PHY register. */
	WRITE_PCIE_REG(PCIE_BK_MGMT_CTRL, (phy_addr & BK_MGMT_CTRL_ADDR_MASK) | BK_MGMT_CTRL_WRITE);

	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = READ_PCIE_REG(PCIE_BK_MGMT_CTRL);
		if (reg & BK_MGMT_CTRL_DONE) {
			return OK;
		}
	}

	pr_err("%s: timeout expired for phy_addr=0x%x\n", __func__, phy_addr);

	/* return error */
	return PCIE_ERROR_VALUE;
}

#ifdef CONFIG_MIPS_BAIKAL_T
#define PLL_WAIT_RETRIES 1000
static int dw_init_pll(const unsigned int pmu_register)
{
	uint32_t reg;
	int i = 0;

	/* Wait for LOCK bit in BK_PMU_COREPLL_CTL */
	while(!(READ_PMU_REG(BK_PMU_COREPLL_CTL) & BK_PMU_LOCK_BIT)) {
		if((i++) == PLL_WAIT_RETRIES) {
			return ERROR;
		}
	}
	/* Set EN & RST bit in pmu_register */
	reg = READ_PMU_REG(pmu_register);
	reg |= BK_PMU_EN_BIT | BK_PMU_RST_BIT;
	WRITE_PMU_REG(pmu_register, reg);

	/* Wait for LOCK bit in pmu_register */
	i = 0;
	while(!(READ_PMU_REG(pmu_register) & BK_PMU_LOCK_BIT)) {
		if((i++) == PLL_WAIT_RETRIES) {
			return ERROR;
		}
	}

	return OK;
}
#endif /* CONFIG_MIPS_BAIKAL_T */

int dw_pcie_init(void)
{
	volatile uint32_t reg;
	uint32_t rstc_mask = 0;
	int i;

	/* PMU PCIe init. */

#ifdef CONFIG_MIPS_BAIKAL_T
	/* Init PCIe PLL only for Baikal-T CPU  */
	/* 2. Start BK_PMU_PCIEPLL_CTL. */
	dw_init_pll(BK_PMU_PCIEPLL_CTL);
#endif /* CONFIG_MIPS_BAIKAL_T */

	/* 3. Read value of BK_PMU_AXI_PCIE_M_CTL, set EN bit. */
	reg = READ_PMU_REG(BK_PMU_AXI_PCIE_M_CTL);
	reg |= PMU_AXI_PCIE_M_CTL_EN;
	WRITE_PMU_REG(BK_PMU_AXI_PCIE_M_CTL, reg);

	/* 4. Read value of BK_PMU_AXI_PCIE_S_CTL, set EN bit. */
	reg = READ_PMU_REG(BK_PMU_AXI_PCIE_S_CTL);
	reg |= PMU_AXI_PCIE_S_CTL_EN;
	WRITE_PMU_REG(BK_PMU_AXI_PCIE_S_CTL, reg);

	/*
	 * 5. Manage RESET* bits
	 * (PHY_RESET, PIPE_RESET, CORE_RST, PWR_RST, STICKY_RST, NONSTICKY_RST)
	 */

	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
#ifdef CONFIG_MIPS_BAIKAL_T1 /* we have Baikal-T1 chip, perform enhanced reset procedure */
	if (reg & PMU_PCIE_RSTC_REQ_PHY_RST)
		rstc_mask |= PMU_PCIE_RSTC_PHY_RESET;
	if (reg & PMU_PCIE_RSTC_REQ_CORE_RST)
		rstc_mask |= PMU_PCIE_RSTC_CORE_RST;
	if (reg & PMU_PCIE_RSTC_REQ_STICKY_RST)
		rstc_mask |= PMU_PCIE_RSTC_STICKY_RST;
	if (reg & PMU_PCIE_RSTC_REQ_NON_STICKY_RST)
		rstc_mask |= PMU_PCIE_RSTC_NONSTICKY_RST;
#else /* we have Baikal-T chip, perform simplified reset procedure */
	rstc_mask = (PMU_PCIE_RSTC_PHY_RESET | PMU_PCIE_RSTC_PIPE_RESET |
		     PMU_PCIE_RSTC_CORE_RST|  PMU_PCIE_RSTC_PWR_RST |
		     PMU_PCIE_RSTC_STICKY_RST | PMU_PCIE_RSTC_NONSTICKY_RST);
#endif /* CONFIG_MIPS_BAIKAL_T1 */
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg | rstc_mask);
	usleep_range(10, 20);
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	reg &= ~rstc_mask;
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg);
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	pr_debug("%s: PCIE_RSTC after reset: %08x (mask was %x)\n", __func__, reg, rstc_mask);
	if (reg & 0x3f11) {
		reg &= ~0x3f11;
		WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg);
		usleep_range(10, 20);
		reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
		pr_debug("%s: new PCIE_RSTC: %08x\n", __func__, reg);
	}

	/* 3.2 Set writing to RO Registers Using DBI */
	WRITE_PCIE_REG(PCIE_MISC_CONTROL_1_OFF, DBI_RO_WR_EN);

	/* set PCI bridge class (subtractive decode) */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg &= ~PMU_PCIE_GENC_DBI2_MODE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);
	WRITE_PCIE_REG(PCIE_TYPE1_CLASS_CODE_REV_ID_REG, 0x06040101);

	/* 3.1 Set DBI2 mode, dbi2_cs = 0x1 */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_DBI2_MODE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	/* 4.1 Allow access to the PHY registers, phy0_mgmt_pcs_reg_sel = 0x1. */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_MGMT_ENABLE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	/*
	 * 7. Wait for stable clocks: SDS_PCS_CLOCK_READY bit in
	 * DWC_GLBL_PLL_MONITOR register of PCIe PHY.
	 */
	for (i = 0; i < PCIE_PHY_RETRIES; i++) {
		reg = dw_pcie_phy_read(PCIE_PHY_DWC_GLBL_PLL_MONITOR);

		if (reg == PCIE_ERROR_VALUE) {
			return ERROR;
		}
		if ((reg & SDS_PCS_CLOCK_READY) == SDS_PCS_CLOCK_READY) {
			break;
		}
	}

	if (i == PCIE_PHY_RETRIES) {
		/* Return an error if the timeout expired. */
		return ERROR;
	}

	/* 
	 * 8. Read value of BK_PMU_PCIE_RSTC, reset bits: PIPE_RESET, CORE_RST,
	 * PWR_RST, STICKY_RST, NONSTICKY_RST, HOT_RESET.
	 */
	reg = READ_PMU_REG(BK_PMU_PCIE_RSTC);
	reg &= ~(PMU_PCIE_RSTC_PIPE_RESET | PMU_PCIE_RSTC_CORE_RST | PMU_PCIE_RSTC_PWR_RST |
		PMU_PCIE_RSTC_STICKY_RST | PMU_PCIE_RSTC_NONSTICKY_RST | PMU_PCIE_RSTC_HOT_RESET);
	WRITE_PMU_REG(BK_PMU_PCIE_RSTC, reg);

	/* 7.1 Disable entire DFE */
	reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_LOOP_CTRL);
	reg |= 0x2;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_LOOP_CTRL, reg);

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_2);
	reg = 0x3F;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_AEQ_VALBBD_2, reg);

	//reg = dw_pcie_phy_read(PCIE_PHY_DWC_RX_AEQ_VALBBD_1);
	reg = 0;
	dw_pcie_phy_write(PCIE_PHY_DWC_RX_AEQ_VALBBD_1, reg);

	/* 9. Set GENx speed in accordance with the Baikal-T(1) SoC rev. */
	reg = READ_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_LINK_CONTROL2_GEN_MASK;
#ifdef CONFIG_MIPS_BAIKAL_T1
	reg |= PCIE_LINK_CONTROL2_GEN3;
#else /* BAIKAL_T */
	reg |= PCIE_LINK_CONTROL2_GEN2;
#endif
	WRITE_PCIE_REG(PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	/* 10. Set LTSSM enable, app_ltssm_enable=0x1 */
	reg = READ_PMU_REG(BK_PMU_PCIE_GENC);
	reg |= PMU_PCIE_GENC_LTSSM_ENABLE;
	WRITE_PMU_REG(BK_PMU_PCIE_GENC, reg);

	return OK;
}
