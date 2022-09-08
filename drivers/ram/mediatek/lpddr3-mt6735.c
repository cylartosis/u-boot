// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek LPDDR3 driver for MT6735 SoC
*/
#include <stdio.h>
#include <stdlib.h>
#include <clk.h>
#include <common.h>
#include <dm.h>
#include <ram.h>
#include <power/regulator.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>

#define EMI_CONA			0x000
#define EMI_CONF			0x028
#define EMI_CONM			0x060
#define EMI_BASE			0x10203000
#define DDRPHY_BASE			0x10213000
#define DRAMC0_BASE			0x10214000
#define DRAMC_NAO_BASE			0x1020e000
#define DRAMC_DDR2CTL_VAL		0xC00642D1
#define EMI_COL_ADDR_MASK		GENMASK(13, 12)
#define EMI_COL_ADDR_SHIFT		12
#define WALKING_PATTERN			0x12345678
#define WALKING_STEP			0x4000000
#define EMI_CONA_VAL			0x0002A052
#define DRAMC_DRVCTL0_VAL		0xaa00aa00
#define DRAMC_DRVCTL1_VAL		0xaa00aa00
#define DRAMC_GDDR3CTL1_VAL		0x01000000
#define DRAMC_ACTIM1_VAL		0x11001330
#define LPDDR3_MODE_REG_63		0x0000003F
#define LPDDR3_MODE_REG_10		0x00FF000A
#define LPDDR3_MODE_REG_1		0x00C30001
#define LPDDR3_MODE_REG_2		0x00080002
#define DRAMC_ACTIM_VAL			0x559A45D6
#define DRAMC_CONF1_VAL			0xF0048683
#define DRAMC_TEST2_3_VAL		0xBF0B0401
#define DRAMC_MISCTL0_VAL		0x17800000
#define DRAMC_ACTIM05T_VAL		0x00001420
#define DRAMC_CONF2_VAL			0x0180AE50
#define DRAMC_PD_CTRL_VAL		0xD1644742

#define r_bias_en_stb_time		(0x00000000 << 24)  //170[31:24]
#define r_bias_lpf_en_stb_time		(0x00000000 << 16)  //170[23:16]
#define r_mempll_en_stb_time		(0x00000000 << 8)   //170[15:8]
#define r_dmall_ck_en_stb_time		(0x00000000 << 0)   //170[7:0]

#define r_dds_en_stb_time		(0x00000000 << 24)  //171[31:24]
#define r_div_en_stb_time		(0x00000000 << 16)  //171[23:16]
#define r_dmpll2_ck_en_stb_time		(0x00000000 << 8)   //171[15:8]
#define r_iso_en_stb_time		(0x00000000 << 0)   //171[7:0]

#define r_bias_en_stb_dis		(0x00000001 << 28)  //172[28]
#define r_bias_en_src_sel		(0x00000001 << 24)  //172[24]
#define r_bias_lpf_en_stb_dis		(0x00000001 << 20)  //172[20]
#define r_bias_lpf_en_src_sel		(0x00000001 << 16)  //172[16]
#define r_mempll4_en_stb_dis		(0x00000001 << 15)  //172[15]
#define r_mempll3_en_stb_dis		(0x00000001 << 14)  //172[14]
#define r_mempll2_en_stb_dis		(0x00000001 << 13)  //172[13]
#define r_mempll_en_stb_dis		(0x00000001 << 12)  //172[12]
#define r_mempll4_en_src_sel		(0x00000001 << 11)  //172[11]
#define r_mempll3_en_src_sel		(0x00000001 << 10)  //172[10]
#define r_mempll2_en_src_sel		(0x00000001 << 9)   //172[9]
#define r_mempll_en_src_sel		(0x00000001 << 8)   //172[8]
#define r_dmall_ck_en_stb_dis		(0x00000001 << 4)   //172[4]
#define r_dmall_ck_en_src_sel		(0x00000001 << 0)   //172[0]

#define r_dds_en_stb_dis		(0x00000001 << 28)  //173[28]
#define r_dds_en_src_sel		(0x00000001 << 24)  //173[24]
#define r_div_en_stb_dis		(0x00000001 << 20)  //173[20]
#define r_div_en_src_sel		(0x00000001 << 16)  //173[16]
#define r_dmpll2_ck_en_stb_dis		(0x00000001 << 12)  //173[12]
#define r_dmpll2_ck_en_src_sel		(0x00000001 << 8)   //173[8]
#define r_iso_en_stb_dis		(0x00000001 << 4)   //173[4]
#define r_iso_en_src_sel		(0x00000001 << 0)   //173[0]

#define r_dmbyp_pll4			(0x00000001 << 0)   //190[0]
#define r_dmbyp_pll3			(0x00000001 << 1)   //190[1]
#define r_dm1pll_sync_mode		(0x00000001 << 2)   //190[2]
#define r_dmall_ck_en			(0x00000001 << 4)   //190[4]
#define r_dmpll2_clk_en			(0x00000001 << 5)   //190[5]

#define pllc1_postdiv_1_0		(0x00000003 << 14)  //180[15:14]
#define pllc1_blp			(0x00000001 << 12)  //180[12]
#define pllc1_mempll_n_info_chg		(0x00000001 << 0)   //189[0]
#define pllc1_dmss_pcw_ncpo_30_0	(0x7fffffff << 1)   //189[31:1]
#define pllc1_mempll_div_en		(0x00000001 <<24)   //181[24]
#define pllc1_mempll_div_6_0		(0x0000007f <<25)   //181[31:25]
#define pllc1_mempll_reserve_2		(0x00000001 <<18)   //181[18]
#define pllc1_mempll_top_reserve_2_0	(0x00000000 <<16)   //182[18:16]
#define pllc1_mempll_bias_en		(0x00000001 <<14)   //181[14]
#define pllc1_mempll_bias_lpf_en	(0x00000001 <<15)   //181[15]
#define pllc1_mempll_en			(0x00000001 << 2)   //180[2]
#define pllc1_mempll_sdm_prd_1		(0x00000001 <<11)   //188[11]

#define mempll2_prediv_1_0		(0x00000000 << 0)   //182[1:0]
#define mempll2_vco_div_sel		(0x00000001 <<29)   //183[29]
#define mempll2_m4pdiv_1_0		(0x00000003 <<10)   //183[11:10],P9
#define mempll2_fbdiv_6_0		(0x0000007f << 2)   //182[8:2],P6
#define mempll2_fb_mck_sel		(0x00000001 << 9)   //183[9]
#define mempll2_fbksel_1_0		(0x00000003 <<10)   //182[11:10]
#define mempll2_bp_br			(0x00000003 <<26)   //183[27:26]
#define mempll2_posdiv_1_0		(0x00000000 <<30)   //183[31:30]
#define mempll2_ref_dl_4_0		(0x00000000 <<27)   //184[31:27]
#define mempll2_fb_dl_4_0		(0x00000000 <<22)   //184[26:22]
#define mempll2_en			(0x00000001 <<18)   //183[18]

#define mempll3_prediv_1_0		(0x00000000 << 0)   //184[1:0]
#define mempll3_vco_div_sel		(0x00000001 <<29)   //185[29]
#define mempll3_m4pdiv_1_0		(0x00000003 <<10)   //185[11:10]
#define mempll3_fbdiv_6_0		(0x0000007f << 2)   //184[8:2]
#define mempll3_bp_br			(0x00000003 <<26)   //185[27:26]
#define mempll3_fb_mck_sel		(0x00000001 << 9)   //185[9]
#define mempll3_fbksel_1_0		(0x00000003 <<10)   //184[11:10]
#define mempll3_posdiv_1_0		(0x00000000 <<30)   //185[31:30]
#define mempll3_ref_dl_4_0		(0x00000000 <<27)   //186[31:27]
#define mempll3_fb_dl_4_0		(0x00000000 <<22)   //186[26:22]
#define mempll3_en			(0x00000001 <<18)   //185[18]

#define mempll4_prediv_1_0		(0x00000000 << 0)   //186[1:0]
#define mempll4_vco_div_sel		(0x00000001 <<29)   //187[29]
#define mempll4_m4pdiv_1_0		(0x00000003 <<10)   //187[11:10]
#define mempll4_fbdiv_6_0		(0x0000007f << 2)   //186[8:2]
#define mempll4_fbksel_1_0		(0x00000003 <<10)   //186[11:10]
#define mempll4_bp_br			(0x00000003 <<26)   //187[27:26]
#define mempll4_fb_mck_sel		(0x00000001 << 9)   //187[9]
#define mempll4_posdiv_1_0		(0x00000000 <<30)   //187[31:30]
#define mempll4_ref_dl_4_0		(0x00000000 <<27)   //188[31:27]
#define mempll4_fb_dl_4_0		(0x00000000 <<22)   //188[26:22]
#define mempll4_en			(0x00000001 <<18)   //187[18]

enum {
	PLL_MODE_1 = 1,
	PLL_MODE_2 = 2,
	PLL_MODE_3 = 3,
};
enum {
	 DDR533 = 533,
	 DDR800 = 800,
	 DDR900 = 900,
	 DDR938 = 938,
	 DDR1066 = 1066,
	 DDR1280 = 1280,
	 DDR1313 = 1313,
	 DDR1333 = 1333,
	 DDR1466 = 1466,
	 DDR1600 = 1600,
};

struct mtk_lpddr3_priv {
	fdt_addr_t emi;
	fdt_addr_t ddrphy_base;
	fdt_addr_t dramc0_base;
	fdt_addr_t dramc_nao_base;
	struct clk phy;
	struct clk phy_mux;
	struct clk mem;
	struct clk mem_mux;
};

unsigned int drvn;
unsigned int drvp;

void ett_rextdn_sw_calibration(struct udevice *dev)
{
	struct mtk_lpddr3_priv *priv = dev_get_priv(dev);
	unsigned int tmp;
	unsigned int sel;
	unsigned int drvn;
	unsigned int drvp;
	printf("Start REXTDN SW calibration...\n");

	/* 1.initialization */

	/* disable power down mode */
	// NA for D123
	/*
	tmp = (readl(priv->ddrphy_base + 0x01e4)&0xffffdfff);
	writel(tmp, priv->ddrphy_base + 0x01e4);

	printf("PD 0x01e4[13]:%xh\n",readl(priv->ddrphy_base + 0x01e4));
	*/

	/* 2.DRVP calibration */

	/* enable P drive calibration */
	// CAL_ENP = 1
	// 0x0644[9] (maoauo, reg map not corrent)
	tmp = (readl(priv->ddrphy_base + 0x0644) |0x00000200);
	writel(tmp, priv->ddrphy_base + 0x0644);

	printf("enable P drive (initial settings), 0x0644:%xh\n", readl(priv->ddrphy_base + 0x0644));

	// delay 1us
	udelay(1);

	/* Start P drive calibration */
	for(drvp = 0 ; drvp <=15; drvp ++)
	{
		tmp = (readl(priv->dramc0_base + 0x00c0)&0xffff0fff)|(drvp << 12);
		writel(tmp, priv->dramc0_base + 0x00c0);

		printf("2.1. DRVP 0x00c0[15:12]:%xh\n", tmp);

		/* wait at least 100ns */
		udelay(1);

		/* check the 3dc[31] from 0 to 1 */

		printf("2.2. CMPOP 0x03dc[31]:%xh\n", readl(priv->ddrphy_base + 0x03dc));

		if ((readl(priv->ddrphy_base + 0x3dc) >> 31)  == 1)
		{
			printf("P drive:%d\n",drvp);
			break;
		}
	}

	// CAL_ENP = 0
	tmp = (readl(priv->ddrphy_base + 0x0644) & 0xfffffdff);
	writel(tmp, priv->ddrphy_base + 0x0644);

	if (drvp == 16)
	{
		drvp = 10; // back to HW defult

		printf("No valid P drive\n");
	}

	/* 3.DRVN calibration */
	/* enable N drive calibration */

	/* Enable N drive calibration */
	// CAL_ENN = 1
	// 0x0644[8] (maoauo, reg map not corrent)
	tmp = (readl(priv->ddrphy_base + 0x0644) |0x00000100);
	writel(tmp, priv->ddrphy_base + 0x0644);

	printf("enable N drive (initial settings), 0x0644:%xh\n", readl(priv->ddrphy_base + 0x0644));

	// delay 1us
	udelay(1);

	/*Start N drive calibration*/
	for(drvn = 0 ; drvn <=15; drvn ++)
	{
		tmp = (readl(priv->dramc0_base + 0x00c0)&0xfffff0ff)|(drvn<<8);
		writel(tmp, priv->dramc0_base + 0x00c0);

		printf("4.1. DRVN 0x00c0[11:8]:%xh\n", tmp);

		// wait at least 100ns
		udelay(1);

		/* check the 3dc[30] from 0 to 1 */

		printf("4.2.CMPON 0x3dc[30]:%xh\n", readl(priv->ddrphy_base + 0x3dc));

		if ((readl(priv->ddrphy_base + 0x3dc) >> 30)  == 1)
		{
			/* fixup the drvn by minus 1 */
			if (drvn > 0)
				drvn--;

			printf("N drive:%d\n",drvn);

			break;
		}
	}

	// CAL_ENN = 0
	tmp = (readl(priv->ddrphy_base + 0x0644) & 0xfffffeff);
	writel(tmp, priv->ddrphy_base + 0x0644);

	if (drvn == 16)
	{
		drvn = 10; // back to default

		printf("No valid N drive\n");
	}

	printf("drvp=%d,drvn=%d\n",drvp,drvn);
}

//#ifdef CONFIG_SPL_BUILD
static int mtk_lpddr3_rank_size_detect(struct udevice *dev)
{
	struct mtk_lpddr3_priv *priv = dev_get_priv(dev);
	int step;
	u32 start, test;
	printf("mtk_lpddr3_rank_size_detect start\n");
	/* To detect size, we have to make sure it's single rank
	 * and it has maximum addressing region
	 */

	writel(WALKING_PATTERN, CONFIG_SYS_SDRAM_BASE);

	if (readl(CONFIG_SYS_SDRAM_BASE) != WALKING_PATTERN)
		printf("SDRAM_BASE return = 0x%x\n", readl(CONFIG_SYS_SDRAM_BASE));
		return -EINVAL;

	for (step = 0; step < 5; step++) {
		writel(~WALKING_PATTERN, (unsigned long)CONFIG_SYS_SDRAM_BASE + (WALKING_STEP << step));

		start = readl(CONFIG_SYS_SDRAM_BASE);
		test = readl((unsigned long)CONFIG_SYS_SDRAM_BASE + (WALKING_STEP << step));
		if ((test != ~WALKING_PATTERN) || test == start)
			break;
	}

	step = step ? step - 1 : 3;
	clrsetbits_le32(priv->emi + EMI_CONA, EMI_COL_ADDR_MASK,
			step << EMI_COL_ADDR_SHIFT);

	printf("mtk_lpddr3_rank_size_detect end\n");
	return 0;
}

void mtk_mempll_init(int type, int pll_mode)
{
	unsigned int temp;

	/*********************************
	* (1) Setup DDRPHY operation mode
	**********************************/

	*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) |= 0x00000001; //DFREQ_DIV2=1
	*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) |= 0x00000001;

	if (pll_mode == PLL_MODE_3)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010020;    //3PLL sync mode, OK
	}
	else if (pll_mode== PLL_MODE_2)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010022;
	}
	else // 1 PLL mode
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00000007;    //1PLL sync mode, OK.
	}

	/*****************************************************************************************
	* (2) Setup MEMPLL operation case & frequency. May set according to dram type & frequency
	******************************************************************************************/
	*((volatile unsigned *)(DDRPHY_BASE + (0x0170 <<2))) = r_bias_en_stb_time | r_bias_lpf_en_stb_time | r_mempll_en_stb_time | r_dmall_ck_en_stb_time;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0171 <<2))) = r_dds_en_stb_time | r_div_en_stb_time | r_dmpll2_ck_en_stb_time | r_iso_en_stb_time;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0172 <<2))) = r_bias_en_stb_dis| r_bias_en_src_sel | r_bias_lpf_en_stb_dis| r_bias_lpf_en_src_sel | r_mempll4_en_stb_dis| r_mempll3_en_stb_dis| r_mempll2_en_stb_dis| r_mempll_en_stb_dis| r_mempll4_en_src_sel | r_mempll3_en_src_sel | r_mempll2_en_src_sel | r_mempll_en_src_sel | r_dmall_ck_en_stb_dis | r_dmall_ck_en_src_sel;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0173 <<2))) = r_dds_en_stb_dis| r_dds_en_src_sel | r_div_en_stb_dis| r_div_en_src_sel | r_dmpll2_ck_en_stb_dis| r_dmpll2_ck_en_src_sel | r_iso_en_stb_dis| r_iso_en_src_sel;

	// MEMPLL common setting
	*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) & (~pllc1_postdiv_1_0)) | 0x00000000; //RG_MEMPLL_POSDIV[1:0] = 2'b00;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) & (~pllc1_blp)) | (0x00000001 << 12); //RG_MEMPLL_BLP = 1'b1;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) & (~pllc1_mempll_div_6_0)) | (0x00000052 << 25); //RG_MEMPLL_DIV = 7'h52;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) & (~pllc1_mempll_reserve_2)) | (0x00000001 << 18); //RG_MEMPLL_RESERVE[2] = 1;

	*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbksel_1_0)) | 0x00000000; //RG_MEMPLL2_FBKSEL[1:0] = 2'b00;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_bp_br)) | (0x00000003 << 26); //RG_MEMPLL2_BP = 1, RG_MEMPLL2_BR=1;

	if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_fb_mck_sel)) | (0x00000001 << 9); //RG_MEMPLL2_FB_MCK_SEL;
	}

	*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbksel_1_0)) | 0x00000000; //RG_MEMPLL3_FBKSEL = 2'b00;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_bp_br)) | (0x00000003 << 26); //RG_MEMPLL3_BP = 1, RG_MEMPLL3_BR=1;

	if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_fb_mck_sel)) | (0x00000001 << 9); //RG_MEMPLL3_FB_MCK_SEL = 1;
	}

	*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbksel_1_0)) | 0x00000000; //RG_MEMPLL4_FBKSEL = 2'b00;
	*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_bp_br)) | (0x00000003 << 26); //RG_MEMPLL4_BP = 1, RG_MEMPLL4_BR=1;

	if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_fb_mck_sel)) | (0x00000001 << 9); //RG_MEMPLL4_FB_MCK_SEL = 1;
	}

	//MEMPLL different setting for different frequency begin
	if (type == DDR1280) // real DDR-1280 (sign-off)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;
		// DDR-1333
		//*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x50d8fe7c << 1); //RG_DMSS_PCW_NCPO[30:0]
		// DDR-1280
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x4da21535 << 1); //RG_DMSS_PCW_NCPO[30:0]

		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL2_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL3_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL4_FBDIV = 7'h0d;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL2_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL3_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL4_FBDIV = 7'h34;
		}
	}
	else if (type == DDR1333) // real DDR-1333
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;
		// DDR-1333
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x50d8fe7c << 1); //RG_DMSS_PCW_NCPO[30:0]
		// DDR-1280
		//*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x4da21535 << 1); //RG_DMSS_PCW_NCPO[30:0]

		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL2_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL3_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL4_FBDIV = 7'h0d;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL2_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL3_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL4_FBDIV = 7'h34;
		}
	}
	else if (type == DDR938) // for DVFS_low (DVS HQA), the same settings as DDR-1333 other than NCPO
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;

		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x38e3f9f0 << 1); //RG_DMSS_PCW_NCPO[30:0]
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x287442a6 << 1); //RG_DMSS_PCW_NCPO[30:0]

		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL2_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL3_FBDIV = 7'h0d;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000d << 2); //RG_MEMPLL4_FBDIV = 7'h0d;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL2_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL3_FBDIV = 7'h34;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000034 << 2); //RG_MEMPLL4_FBDIV = 7'h34;
		}
	}
	else if (type == DDR1466)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x52902d02 << 1); //RG_DMSS_PCW_NCPO[30:0] = 31'h52902d02;
		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL2_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL3_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL4_FBDIV = 7'h0e;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL2_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL3_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL4_FBDIV = 7'h38;
		}
	}
	else if (type == DDR1313) // for Denali-3, DVFS-low frequency, the same settings as DDR-1466 other than NCPO
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x49F24924 << 1); //RG_DMSS_PCW_NCPO[30:0] = 31'h49F24924;
		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL2_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL3_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL4_FBDIV = 7'h0e;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL2_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL3_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL4_FBDIV = 7'h38;
		}
	}
	else if (type == DDR1600) // for Denali-3, DVFS-high frequency, the same settings as DDR-1466 other than NCPO
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_vco_div_sel)) | 0x00000000; //RG_MEMPLL2_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_vco_div_sel)) | 0x00000000; //RG_MEMPLL3_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_vco_div_sel)) | 0x00000000; //RG_MEMPLL4_VCO_DIV_SEL =0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x5A1C21C2 << 1); //RG_DMSS_PCW_NCPO[30:0] = 31'h5A1C21C2;
		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL2_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL3_FBDIV = 7'h0e;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000e << 2); //RG_MEMPLL4_FBDIV = 7'h0e;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL2_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL3_FBDIV = 7'h38;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x00000038 << 2); //RG_MEMPLL4_FBDIV = 7'h38;
		}
	}
	else // DDR-1066
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0189 <<2))) & (~pllc1_dmss_pcw_ncpo_30_0)) | (0x4c68b439 << 1); //RG_DMSS_PCW_NCPO[30:0] = 31'h4c68b439;
		if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000000b << 2); //RG_MEMPLL2_FBDIV = 7'h0b;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000000b << 2); //RG_MEMPLL3_FBDIV = 7'h0b;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000000b << 2); //RG_MEMPLL4_FBDIV = 7'h0b;
		}
		else
		{
			*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0182 <<2))) & (~mempll2_fbdiv_6_0)) | (0x0000002c << 2); //RG_MEMPLL2_FBDIV = 7'h2c;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0184 <<2))) & (~mempll3_fbdiv_6_0)) | (0x0000002c << 2); //RG_MEMPLL3_FBDIV = 7'h2c;
			*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0186 <<2))) & (~mempll4_fbdiv_6_0)) | (0x0000002c << 2); //RG_MEMPLL4_FBDIV = 7'h2c;
		}
	}
	//MEMPLL different setting for different frequency end

	if (pll_mode == PLL_MODE_2)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010022  | r_dmpll2_clk_en;
	}
	else if (pll_mode == PLL_MODE_3)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010020 | r_dmpll2_clk_en;    //3PLL sync mode
	}
	else // 1-PLL mode
	{
		//*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00000007 | r_dmpll2_clk_en;    //1PLL sync mode
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00000007 ;    //1PLL sync mode
	}

	/***********************************
	* (3) Setup MEMPLL power on sequence
	************************************/

	udelay(2);

	*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) & (~pllc1_mempll_bias_en)) | (0x00000001 << 14); //RG_MEMPLL_BIAS_EN = 1'b1;

	udelay(2);

	*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) & (~pllc1_mempll_bias_lpf_en)) | (0x00000001 << 15); //RG_MEMPLL_BIAS_LPF_EN = 1'b1;

	udelay(1000);

	*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0180 <<2))) & (~pllc1_mempll_en)) | (0x00000001 << 2); //RG_MEMPLL_EN = 1'b1;

	udelay(20);

	*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0181 <<2))) & (~pllc1_mempll_div_en)) | (0x00000001 << 24); //RG_MEMPLL_DIV_EN = 1'b1;

	udelay(1);

	if (pll_mode == PLL_MODE_3)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_en)) | (0x00000001 << 18); //RG_MEMPLL2_EN = 1'b1;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_en)) | (0x00000001 << 18); //RG_MEMPLL3_EN = 1'b1;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_en)) | (0x00000001 << 18); //RG_MEMPLL4_EN = 1'b1;
	}
	else if (pll_mode == PLL_MODE_2)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_en)) | (0x00000001 << 18); //RG_MEMPLL2_EN = 1'b1;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_en)) | (0x00000000 << 18); //RG_MEMPLL3_EN = 1'b0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_en)) | (0x00000001 << 18); //RG_MEMPLL4_EN = 1'b1;
	}
	else
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0183 <<2))) & (~mempll2_en)) | (0x00000001 << 18); //RG_MEMPLL2_EN = 1'b1;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0185 <<2))) & (~mempll3_en)) | (0x00000000 << 18); //RG_MEMPLL3_EN = 1'b0;
		*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) = (*((volatile unsigned *)(DDRPHY_BASE + (0x0187 <<2))) & (~mempll4_en)) | (0x00000000 << 18); //RG_MEMPLL4_EN = 1'b0;
	}

	udelay(23);

	if (pll_mode == PLL_MODE_2)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010022  | r_dmpll2_clk_en | r_dmall_ck_en;
	}
	else if (pll_mode == PLL_MODE_3)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00010020 | r_dmpll2_clk_en | r_dmall_ck_en;    //3PLL sync mode
	}
	else // 1-PLL mode
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0190 <<2))) = 0x00000007 | r_dmpll2_clk_en | r_dmall_ck_en;    //1PLL sync mode
	}

	/**********************************
	* (4) MEMPLL control switch to SPM
	***********************************/
	if ((pll_mode == PLL_MODE_3) || (pll_mode == PLL_MODE_2))
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0173 <<2))) = 0x40101000;   //[0]ISO_EN_SRC=0,[22]DIV_EN_SC_SRC=0 (pll2off),[16]DIV_EN_SRC=0,[8]PLL2_CK_EN_SRC=1(1pll),[8]PLL2_CK_EN_SRC=0(3pll)
	}
	else // 1-PLL mode
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0173 <<2))) = 0x40101000;   //[0]ISO_EN_SRC=0,[22]DIV_EN_SC_SRC=0 (pll2off),[16]DIV_EN_SRC=0,[8]PLL2_CK_EN_SRC=1(1pll),[8]PLL2_CK_EN_SRC=0(3pll)
	}

	if (pll_mode == PLL_MODE_3)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0172 <<2))) = 0x0000F010;   //[24]BIAS_EN_SRC=0,[16]BIAS_LPF_EN_SRC=0,[8]MEMPLL_EN,[9][10][11]MEMPLL2,3,4_EN_SRC,[0]ALL_CK_EN_SRC=0
	}
	else if (pll_mode == PLL_MODE_2)
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0172 <<2))) = 0x0000F410;   //PLL3 switch to SW control
	}
	else // 1-PLL mode
	{
		*((volatile unsigned *)(DDRPHY_BASE + (0x0172 <<2))) = 0x0000FC10;   //1PLL mode, MEMPLL3,4_EN no change to spm controller.eep to 1'b0 for power saving.
		//*((volatile unsigned *)(DDRPHY_BASE + (0x0172 <<2))) = 0x0000F010; // sim ok
	}

	//*((volatile unsigned *)(DDRPHY_BASE + (0x0170 <<2))) = 0x003C1B96;   //setting for delay time
	*((volatile unsigned *)(DDRPHY_BASE + (0x0170 <<2))) = 0x063C0000;   //setting for delay time

// MEMPLL configuration end
}

static int init_lpddr3(struct udevice *dev)
{
	struct mtk_lpddr3_priv *priv = dev_get_priv(dev);
	int i, ret;

	ret = clk_set_parent(&priv->phy, &priv->phy_mux);
	if (ret)
		return ret;
		*((volatile unsigned *)(EMI_BASE + 0x0040)) |= 0xcc00;
		ett_rextdn_sw_calibration(dev);
		*((volatile unsigned *)(EMI_BASE+0x00000000))= 0x2a052;
		*((volatile unsigned *)(EMI_BASE+0x00000100))= 0x7f07704a;
		*((volatile unsigned *)(EMI_BASE+0x00000108))= 0xa0a070db;
		*((volatile unsigned *)(EMI_BASE+0x00000110))= 0xa0a07042;
		*((volatile unsigned *)(EMI_BASE+0x00000118))= 0x07007047;
		*((volatile unsigned *)(EMI_BASE+0x00000120))= 0x2030604b;
		*((volatile unsigned *)(EMI_BASE+0x00000128))= 0xa0a07046;
		*((volatile unsigned *)(EMI_BASE+0x00000134))= 0xa0a07046; // Please set 0x134=0x128

		*((volatile unsigned *)(EMI_BASE+0x00000008))=0x0d1e293a;
		*((volatile unsigned *)(EMI_BASE+0x00000010))=0x09190819;
		*((volatile unsigned *)(EMI_BASE+0x00000030))=0x2b2b282e;


		*((volatile unsigned *)(EMI_BASE+0x00000018))=0x3657587a; //SMI threthold
		*((volatile unsigned *)(EMI_BASE+0x00000020))=0xFFFF0848;
		//*((volatile unsigned *)(EMI_BASE+0x00000038))=0x00000000;

		*((volatile unsigned *)(EMI_BASE+0x00000078))=0x34220e17;// defer ultra excpt MDMCU   //lower MDMCU perfer setting A


		*((volatile unsigned *)(EMI_BASE+0x000000d0))=0xCCCCCCCC;//R/8 W/8 outstanding
		*((volatile unsigned *)(EMI_BASE+0x000000d8))=0xcccccccc;//R/8 W/8 outstanding

		*((volatile unsigned *)(EMI_BASE+0x000000e8))=0x00020027;// LPDDR3 //lower MDMCU perfer setting A

		*((volatile unsigned *)(EMI_BASE+0x000000f0))=0x38460000; //[16] 0:disable urgent read

		*((volatile unsigned *)(EMI_BASE+0x000000f8))=0x00000000;

		*((volatile unsigned *)(EMI_BASE+0x00000140))=0x20406188;//0x12202488;   // 83 for low latency
		*((volatile unsigned *)(EMI_BASE+0x00000144))=0x20406188;//0x12202488; //new add
		*((volatile unsigned *)(EMI_BASE+0x00000148))=0x9719595e;//0323 chg, ori :0x00462f2f
		*((volatile unsigned *)(EMI_BASE+0x0000014c))=0x9719595e; // new add
		*((volatile unsigned *)(EMI_BASE+0x00000150))=0x64f3fc79;
		*((volatile unsigned *)(EMI_BASE+0x00000154))=0x64f3fc79;

		*((volatile unsigned *)(EMI_BASE+0x00000158))=0xff01ff00;// ???????????????????????0x08090800;

		//==============Scramble address==========================
		*((volatile unsigned *)(EMI_BASE+0x00000028))=0x00421000;
		*((volatile unsigned *)(EMI_BASE+0x00000060))=0x000006ff;
		//===========END===========================================
		// CLK ([27:24]/CS1 ([31:28]) delay
		*((volatile unsigned *)(DRAMC0_BASE + 0x000c)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x000c)) = 0x00000000;

		// TX driving 0x0a
		*((volatile unsigned *)(DRAMC0_BASE + 0x00b4)) = 0x99009900; // DQC (@ DRAMC)
		*((volatile unsigned *)(DDRPHY_BASE + 0x00b4)) = 0x99009900;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00b8)) = 0x99009900; // DQ (@ DRAMC)
		*((volatile unsigned *)(DDRPHY_BASE + 0x00b8)) = 0x99009900;

		if ((10>1) && (10>1)) // drv -1 for large jitter. ONLY for D-1
		{
			*((volatile unsigned *)(DRAMC0_BASE + 0x00bc)) = 0x99009900;  // CMD (@ DDRPHY)
			*((volatile unsigned *)(DDRPHY_BASE + 0x00bc)) = 0x99009900;
		}
		else
		{
			*((volatile unsigned *)(DRAMC0_BASE + 0x00bc)) = 0x99009900;  // CMD (@ DDRPHY)
			*((volatile unsigned *)(DDRPHY_BASE + 0x00bc)) = 0x99009900;
		}
		// Pre-emphasis ON. Should be set after MEMPLL init.
		// No Pre-emphasis for D123
		//*((volatile unsigned *)(DRAMC0_BASE + 0x0640)) |= 0x00003f00;
		//*((volatile unsigned *)(DDRPHY_BASE + 0x0640)) |= 0x00003f00;

		// Default set to external Vref, 0x644[0]=0
		*((volatile unsigned *)(DRAMC0_BASE + 0x0644)) &= 0xfffffffe;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0644)) &= 0xfffffffe;


		*((volatile unsigned *)(DRAMC0_BASE + 0x0048)) = 0x0001110d; // [16] XTALK
		*((volatile unsigned *)(DDRPHY_BASE + 0x0048)) = 0x0001110d;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00d8)) = 0x00500900; //Edward: 100b for LPDDR2/3, 110b for DDR3x16x2
		*((volatile unsigned *)(DDRPHY_BASE + 0x00d8)) = 0x00500900;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00e4)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) = 0x00000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x008c)) = 0x00000001;
		*((volatile unsigned *)(DDRPHY_BASE + 0x008c)) = 0x00000001;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0090)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0090)) = 0x00000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0094)) = 0x80000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0094)) = 0x80000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00dc)) = 0x83004004;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00dc)) = 0x83004004;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) = 0x1c004004; // gating coarse
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) = 0x1c004004;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xaa080033;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa080033;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) = 0xc0000000; // [30] pinmux, 1:LPDDR3, 0: LPDDR2. [31] Enable 4-bit MUX
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) = 0xc0000000; // [30] pinmux, 1:LPDDR3, 0: LPDDR2. [31] Enable 4-bit MUX

		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) = DRAMC_GDDR3CTL1_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) = DRAMC_GDDR3CTL1_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 0x00f00000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 0x00f00000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0168)) = 0x00000080;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0168)) = 0x00000080;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00d8)) = 0x00700900;   // Edward : 100b for LPDDR2/3, 110b for DDR3x16x2
		*((volatile unsigned *)(DDRPHY_BASE + 0x00d8)) = 0x00700900;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0028)) = 0xf1200f01;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0028)) = 0xf1200f01;

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e0)) = 0x2001ebff;    //LPDDR2EN set to 0
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e0)) = 0x2001ebff;    //LPDDR2EN set to 0
		//}

		// Darren
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e8)) = DRAMC_ACTIM1_VAL;    //LPDDR3EN set to 1
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e8)) = DRAMC_ACTIM1_VAL;    //LPDDR3EN set to 1
		// reg map N/A (D123 has no this option. Force 4-bit swap)
		*((volatile unsigned *)(DRAMC0_BASE + 0x0158)) = 0xf0f0f0f0;      // Edward: 4 bit swap enable 0xf0f0f0f0, disable 0xff00ff00
		*((volatile unsigned *)(DDRPHY_BASE + 0x0158)) = 0xf0f0f0f0;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0400)) = 0x00111100;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0400)) = 0x00111100;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0404)) = 0x00000002;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0404)) = 0x00000002;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0408)) = 0x00222222;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0408)) = 0x00222222;

		*((volatile unsigned *)(DRAMC0_BASE + 0x040c)) = 0x33330000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x040c)) = 0x33330000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0410)) = 0x33330000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0410)) = 0x33330000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) = 0x0b052311;              //Edward: cross rank timing.
		*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) = 0x0b052311;

		// enable clock here for tINIT2
		*((volatile unsigned *)(DRAMC0_BASE + 0x01dc)) |= 0x04000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01dc)) |= 0x04000000;
		udelay(1);

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e4)) = 0x00000005;   //CKEBYCTL
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) = 0x00000005;   //CKEBYCTL

		udelay(200);//Wait > 200us              // Edward : add this according to 6589.

		// Darren
		*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = LPDDR3_MODE_REG_63;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = LPDDR3_MODE_REG_63;
		//*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = 0x0000003F;
		//*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = 0x0000003F;

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000001;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000001;

		udelay(10);//Wait > 10us                              // Edward : add this according to DRAM spec, should wait at least 10us if not checking DAI.

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) &= (~0x7); // Edward : Add this to disable  two ranks support for ZQ calibration tempority.
		*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) &= (~0x7);

		// Darren
		*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = LPDDR3_MODE_REG_10;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = LPDDR3_MODE_REG_10;
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000001;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000001;

		udelay(1);            //Wait > 1us. Edward : Add this because tZQINIT min value is 1us.

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000000;

		// Edward : Add this for dual ranks support.
		if ( *(volatile unsigned *)(EMI_CONA)& 0x20000)  {
				//for chip select 1: ZQ calibration
				*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) |= (0x8);
				*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) |= (0x8);

				// Darren
				*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = LPDDR3_MODE_REG_10;
				*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = LPDDR3_MODE_REG_10;


				*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000001;
				*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000001;
			  udelay(1);//Wait > 1us

				*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000000;
				*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000000;

				//swap back
				*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) &= (~0x8);
				*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) &= (~0x8);
				*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) |= (0x1);
				*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) |= (0x1);
		}

		// Darren
		*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = LPDDR3_MODE_REG_1;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = LPDDR3_MODE_REG_1;

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000001;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000001;
		udelay(1);//Wait > 1us                        // Edward : 6589 has this delay. Seems no need.

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000000;

		// Darren
		*((volatile unsigned *)(DRAMC0_BASE + 0x0088)) = LPDDR3_MODE_REG_2;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0088)) = LPDDR3_MODE_REG_2;
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00000001;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00000001;

		udelay(1);//Wait > 1us                        // Edward : 6589 has this delay. Seems no need.

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) = 0x00001100;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) = 0x00001100;

		// Edward : add two rank enable here.
		if ( *(volatile unsigned *)(EMI_CONA)& 0x20000)  {
			*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) = 0x00112391;
			*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) = 0x00112391;
		} else {
			*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) = 0x00112390;
			*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) = 0x00112390;
		}

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e4)) = 0x00000001;  //CKEBYCTL
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) = 0x00000001;  //CKEBYCTL

		*((volatile unsigned *)(DRAMC0_BASE + 0x01ec)) = 0x00000001;    // Edward : Add this to enable dual scheduler according to CC Wen. Should enable this for all DDR type.
		*((volatile unsigned *)(DDRPHY_BASE + 0x01ec)) = 0x00000001;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0084)) = 0x00000a56;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0084)) = 0x00000a56;

		// [27:24] CLK delay. Better to move above
		// [31:28] CS1 delay
		// by JC
		// AC timing, by Chiahsien
		*((volatile unsigned *)(DRAMC0_BASE + 0x0000)) = DRAMC_ACTIM_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0000)) = DRAMC_ACTIM_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0004)) = DRAMC_CONF1_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0004)) = DRAMC_CONF1_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0044)) = DRAMC_TEST2_3_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0044)) = DRAMC_TEST2_3_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) = DRAMC_DDR2CTL_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) = DRAMC_DDR2CTL_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00fc)) = DRAMC_MISCTL0_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00fc)) = DRAMC_MISCTL0_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x01e8)) = DRAMC_ACTIM1_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e8)) = DRAMC_ACTIM1_VAL;

		*((volatile unsigned *)(DDRPHY_BASE + 0x01f8)) = DRAMC_ACTIM05T_VAL;

		// Derping: R_DMREFTHD(0x08[26:24])='h1
		// Fix refresh enable asynchronous issue
		*((volatile unsigned *)(DRAMC0_BASE + 0x0008)) = DRAMC_CONF2_VAL | 0x10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0008)) = DRAMC_CONF2_VAL | 0x10000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x01dc)) = DRAMC_PD_CTRL_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01dc)) = DRAMC_PD_CTRL_VAL;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0008)) = DRAMC_CONF2_VAL;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0008)) = DRAMC_CONF2_VAL;


		*((volatile unsigned *)(DRAMC0_BASE + 0x0010)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0010)) = 0x00000000;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00f8)) = 0xedcb000f;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f8)) = 0xedcb000f;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0020)) = 0x00000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0020)) = 0x00000000;


		if (EMI_CONA_VAL & 0x20000)
		{
			*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) = 0x00114381;      //dsiable R_DMMRS2RK, this bit enable will impact MRR result // Edward : Modify [14:12] XRTR2W 1->2 [10:8] XRTR2R 1->3
			*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) = 0x00114381;      //dsiable R_DMMRS2RK, this bit enable will impact MRR result
		}
		else
		{
			*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) = 0x00114380;      //dsiable R_DMMRS2RK, this bit enable will impact MRR result // Edward : Modify [14:12] XRTR2W 1->2 [10:8] XRTR2R 1->3
			*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) = 0x00114380;      //dsiable R_DMMRS2RK, this bit enable will impact MRR result
		}

		// DLE 8
		*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) = (*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) & 0xFFFFFF8F) | ((8 & 0x07) <<4);
		*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) = (*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) & 0xFFFFFF8F) |  ((8 & 0x07) <<4);

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e4)) = (*((volatile unsigned *)(DRAMC0_BASE + 0x00e4)) & 0xFFFFFFEF) | (((8 >> 3) & 0x01) << 4);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) =  (*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) & 0xFFFFFFEF) | (((8 >> 3) & 0x01) << 4);

		*((volatile unsigned *)(DRAMC0_BASE + 0x0210)) = 0x80B0B07;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0210)) = 0x80B0B07;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0214)) = 0xC0F0B0D;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0214)) = 0xC0F0B0D;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0218)) = 0x60405;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0218)) = 0x60405;
		*((volatile unsigned *)(DRAMC0_BASE + 0x021c)) = 0x7090709;
		*((volatile unsigned *)(DDRPHY_BASE + 0x021c)) = 0x7090709;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0220)) = 0xA0D0A07;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0220)) = 0xA0D0A07;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0224)) = 0xD0F0B0F;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0224)) = 0xD0F0B0F;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0228)) = 0x20101;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0228)) = 0x20101;
		*((volatile unsigned *)(DRAMC0_BASE + 0x022c)) = 0x3020206;
		*((volatile unsigned *)(DDRPHY_BASE + 0x022c)) = 0x3020206;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0x15191B18;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0018)) = 0x15191B18;
		*((volatile unsigned *)(DRAMC0_BASE + 0x001c)) = 0x15191B18;
		*((volatile unsigned *)(DDRPHY_BASE + 0x001c)) = 0x15191B18;

		*((volatile unsigned *)(EMI_BASE + 0x0060)) |= 0x400;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) |= 0x6ff00;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) |= 0x6ff00;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) |= 0x100;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) |= 0x100;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x01e4)) |= 0x100;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) = 0x1c004004;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) = 0x4004;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xff;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa0eff00;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0094)) = 0xb8383838;

		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) &= ~(0x100);
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) &= ~(0x100);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x01e4)) &= ~(0x100);

		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) |= 2000000;
		udelay(100);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) &= ~(2000000);

		*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) = 0x80064101;
		*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) = 0x40000080;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) = 0x11;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x003c)) = 0x55000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x0040)) = 0x3ff;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0008)) = 0x180ae50;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0008)) = 0xd1e293a;
		
		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0210)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0214)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0218)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x021c)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0220)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0224)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0228)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x022c)) = 0;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x003c)) = 0x55000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x0040)) = 0x3ff;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0210)) = 0x80c0a09;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0214)) = 0xd0f0b0e;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0218)) = 0x1040404;
		*((volatile unsigned *)(DRAMC0_BASE + 0x021c)) = 0x8090608;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0220)) = 0xc0c0c09;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0224)) = 0xf0f0e0f;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0228)) = 0x3050405;
		*((volatile unsigned *)(DRAMC0_BASE + 0x022c)) = 0x6070608;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0x1c1c1c1c;
	
		*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) |= (0x8);
		*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) |= (0x8);

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00e0)) |= 0x10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) |= 0x100;
		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) |= 0x100;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x01e4)) |= 0x100;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xff;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa0eff00;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0094)) = 0xb8383838;

		*((volatile unsigned *)(DDRPHY_BASE + 0x01e4)) &= ~(0x100);
		*((volatile unsigned *)(DRAMC0_BASE + 0x01e4)) &= ~(0x100);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x01e4)) &= ~(0x100);

		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) |= 2000000;
		udelay(100);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) &= ~(2000000);

		*((volatile unsigned *)(DRAMC0_BASE + 0x007c)) = 0x80064101;
		*((volatile unsigned *)(DDRPHY_BASE + 0x007c)) = 0x40000080;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e4)) = 0x11;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x003c)) = 0x55000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x0040)) = 0x3ff;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0008)) = 0x180ae50;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0008)) = 0xd1e293a;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0210)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0214)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0218)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x021c)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0220)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0224)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0228)) = 0;
		*((volatile unsigned *)(DRAMC0_BASE + 0x022c)) = 0;

		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x003c)) = 0x55000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x0040)) = 0x3ff;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0210)) = 0x80c0a09;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0214)) = 0xd0f0b0e;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0218)) = 0x1040404;
		*((volatile unsigned *)(DRAMC0_BASE + 0x021c)) = 0x8090608;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0220)) = 0xc0c0c09;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0224)) = 0xf0f0e0f;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0228)) = 0x3050405;
		*((volatile unsigned *)(DRAMC0_BASE + 0x022c)) = 0x6070608;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0x1c1c1c1c;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0118)) = 0x4;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xff;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa0eff00;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0098)) = 0x38383838;

		*((volatile unsigned *)(DRAMC0_BASE + 0x001c)) = 0x1b1b1c1b;

		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) |= 2000000;
		udelay(100);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) &= ~(2000000);

		*((volatile unsigned *)(DRAMC0_BASE + 0x0110)) &= (~0x8);
		*((volatile unsigned *)(DDRPHY_BASE + 0x0110)) &= (~0x8);

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) = 0x1c004004;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) = 0x4004;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xff;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa0eff00;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0094)) = 0xb8383838;

		*((volatile unsigned *)(DRAMC0_BASE + 0x0018)) = 0x1c1c1c1c;

		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) |= 2000000;
		udelay(100);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) &= ~(2000000);

		*((volatile unsigned *)(DDRPHY_BASE + 0x0010)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0200)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0204)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0208)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x020c)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0014)) = 0x10;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0014)) = 0;

		*((volatile unsigned *)(DDRPHY_BASE + 0x0010)) = 0x9190819;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0200)) = 0x1110000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0204)) = 0x11110000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0208)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x020c)) = 0;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0014)) = 0x10;

		*((volatile unsigned *)(DRAMC0_BASE + 0x00e0)) = 0x1c004004;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00e0)) = 0x4004;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0124)) = 0xff;
		*((volatile unsigned *)(DDRPHY_BASE + 0x0124)) = 0xaa0eff00;
		*((volatile unsigned *)(DRAMC0_BASE + 0x0094)) = 0xb8383838;

		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) |= 10000000;
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) |= 2000000;
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) |= 2000000;
		udelay(100);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f0)) &= ~(10000000);
		*((volatile unsigned *)(DDRPHY_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC0_BASE + 0x00f4)) &= ~(2000000);
		*((volatile unsigned *)(DRAMC_NAO_BASE + 0x00f4)) &= ~(2000000);

		for (i = 0; i < 256; i++) {
			printf("writel(0x%x, 0x%x)\n", (readl(priv->emi + 0x00+i*4)), (EMI_BASE + 0x00+i*4));
			printf("writel(0x%x, 0x%x)\n", (readl(priv->dramc0_base + 0x00+i*4)), (DRAMC0_BASE + 0x00+i*4));
			printf("writel(0x%x, 0x%x)\n", (readl(priv->ddrphy_base + 0x00+i*4)), (DDRPHY_BASE + 0x00+i*4));
			printf("writel(0x%x, 0x%x)\n", (readl(priv->ddrphy_base + 0x400+i*4)), (DDRPHY_BASE + 0x400+i*4));
			printf("writel(0x%x, 0x%x)\n", (readl(priv->dramc_nao_base + 0x00+i*4)), (DRAMC_NAO_BASE + 0x00+i*4));
		}

		return mtk_lpddr3_rank_size_detect(dev);

}
static int mtk_lpddr3_probe(struct udevice *dev)
{
	struct mtk_lpddr3_priv *priv = dev_get_priv(dev);

	priv->emi = dev_read_addr_index(dev, 0);
	if (priv->emi == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->ddrphy_base = dev_read_addr_index(dev, 1);
	if (priv->ddrphy_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->dramc0_base = dev_read_addr_index(dev, 2);
	if (priv->dramc0_base == FDT_ADDR_T_NONE)
		return -EINVAL;

	priv->dramc_nao_base = dev_read_addr_index(dev, 3);
	if (priv->dramc0_base == FDT_ADDR_T_NONE)
		return -EINVAL;

//#ifdef CONFIG_SPL_BUILD
	int ret;

	ret = clk_get_by_index(dev, 0, &priv->phy);
	if (ret)
		return ret;

	ret = clk_get_by_index(dev, 1, &priv->phy_mux);
	if (ret)
		return ret;

	ret = clk_get_by_index(dev, 2, &priv->mem);
	if (ret)
		return ret;

	ret = clk_get_by_index(dev, 3, &priv->mem_mux);
	if (ret)
		return ret;

	mtk_mempll_init(DDR1333, PLL_MODE_1);
	ret = init_lpddr3(dev);
	if (ret)
		return ret;
	writel((readl(priv->dramc0_base + 0x01ec) | 0x4f10), priv->dramc0_base + 0x01ec);         // Edward : Need to make sure bit 13 is 0.
	writel((readl(priv->dramc0_base + 0x00fc) & 0xf9ffffff), priv->dramc0_base + 0x00fc);         // Edward : Need to make sure bit 13 is 0.
	writel((readl(priv->dramc0_base + 0x01dc) | 0x02000000), priv->dramc0_base + 0x01dc); // [25] DCMEN=1
//#endif
	return 0;
}

static int mtk_lpddr3_get_info(struct udevice *dev, struct ram_info *info)
{
	struct mtk_lpddr3_priv *priv = dev_get_priv(dev);
	u32 val = readl(priv->emi + EMI_CONA);

	info->base = CONFIG_SYS_SDRAM_BASE;

	switch ((val & EMI_COL_ADDR_MASK) >> EMI_COL_ADDR_SHIFT) {
	case 0:
		info->size = SZ_512M;
		break;
	case 1:
		info->size = SZ_1G;
		break;
	case 2:
		info->size = SZ_2G;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct ram_ops mtk_ddr3_ops = {
	.get_info = mtk_lpddr3_get_info,
};

static const struct udevice_id mtk_ddr3_ids[] = {
	{ .compatible = "mediatek,mt6735-dramc" },
	{ }
};

U_BOOT_DRIVER(mediatek_lpddr3) = {
	.name = "mediatek_lpddr3",
	.id = UCLASS_RAM,
	.of_match = mtk_ddr3_ids,
	.ops = &mtk_ddr3_ops,
	.probe = mtk_lpddr3_probe,
	.priv_auto sizeof(struct mtk_lpddr3_priv),
};
