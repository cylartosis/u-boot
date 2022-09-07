// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 MediaTek Inc.
 * Copyright (C) 2019 BayLibre, SAS
 * Author: Fabien Parent <fparent@baylibre.com>
 */

#include <clk.h>
#include <common.h>
#include <cpu_func.h>
#include <dm.h>
#include <fdtdec.h>
#include <init.h>
#include <ram.h>
#include <linux/io.h>
#include <asm/arch/misc.h>
#include <asm/armv8/mmu.h>
#include <asm/cache.h>
#include <asm/global_data.h>
#include <asm/sections.h>
#include <dm/uclass.h>
#include <dt-bindings/clock/mt6735-clk.h>
#include <configs/mt6735.h>
#include <pwrap/pwrap.h>

#define VER_BASE		0x08000000
#define VER_SIZE		0x10

#define APHW_CODE		0x00
#define APHW_SUBCODE		0x04
#define APHW_VER		0x08
#define APSW_VER		0x0c

#define WDOG_RESTART		0x10212008
#define WDOG_RESTART_KEY	0x1971
#define WDOG_SWRST		0x10212014
#define WDOG_SWRST_KEY		0x1209

DECLARE_GLOBAL_DATA_PTR;

static struct mm_region mt6735_mem_map[] = {
	{
		.virt = 0x40000000UL,
		.phys = 0x40000000UL,
		.size = 0xc0000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) | PTE_BLOCK_OUTER_SHARE,
	}, {
		.virt = 0x00000000UL,
		.phys = 0x00000000UL,
		.size = 0x40000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		0,
	}
};
struct mm_region *mem_map = mt6735_mem_map;

int mtk_pll_early_init(void)
{
	unsigned long pll_rates[] = {
		[CLK_APMIXED_ARMPLL] = 793000000,
		[CLK_APMIXED_MAINPLL] = 1092000000,
		[CLK_APMIXED_UNIVPLL] = 1248000000,
		[CLK_APMIXED_MMPLL] = 450000000,
		[CLK_APMIXED_MSDCPLL] = 800000000,
		[CLK_APMIXED_TVDPLL] = 176000000,
		[CLK_APMIXED_VENCPLL] = 1183000000,
		[CLK_APMIXED_APLL1] = 90316800,
		[CLK_APMIXED_APLL2] = 90316800,
	};
	struct udevice *dev;
	int ret, i;

	ret = uclass_get_device_by_driver(UCLASS_CLK,
			DM_DRIVER_GET(mtk_clk_apmixedsys), &dev);
	if (ret)
		return ret;

	/* configure default rate then enable apmixedsys */
	for (i = 0; i < ARRAY_SIZE(pll_rates); i++) {
		struct clk clk = { .id = i, .dev = dev };

		ret = clk_set_rate(&clk, pll_rates[i]);
		if (ret)
			return ret;

		ret = clk_enable(&clk);
		if (ret)
			return ret;
	}

	return 0;
}

int mtk_soc_early_init(void)
{
	struct udevice *dev;
	int ret;

	/* initialize early clocks */
	ret = mtk_pll_early_init();
	if (ret)
		return ret;

	mtk_pwrap_init(dev);

	ret = uclass_first_device_err(UCLASS_RAM, &dev);
	if (ret)
		return ret;

	return 0;
}

#ifdef CONFIG_TARGET_SPL_MT6735
int dram_init(void)
{
	struct ram_info ram;
	struct udevice *dev;
	int ret;

	ret = uclass_first_device_err(UCLASS_RAM, &dev);
	if (ret)
		return ret;

	ret = ram_get_info(dev, &ram);
	if (ret)
		return ret;

	debug("RAM init base=%llx, size=%lx\n", ram.base, ram.size);

	gd->ram_size = ram.size;

	return 0;
}
#else
int dram_init(void)
{
	int ret;

	ret = fdtdec_setup_memory_banksize();
	if (ret)
		return ret;
	return fdtdec_setup_mem_size_base();
}
#endif /* CONFIG_TARGET_SPL_MT6735 */

void reset_cpu(void)
{
	writel(WDOG_RESTART_KEY, WDOG_RESTART);
	writel(WDOG_SWRST_KEY, WDOG_SWRST);
	hang();
}

int print_cpuinfo(void)
{
	void __iomem *chipid;
	u32 hwcode, swver;

	chipid = ioremap(VER_BASE, VER_SIZE);
	hwcode = readl(chipid + APHW_CODE);
	swver = readl(chipid + APSW_VER);

	printf("CPU:   MediaTek MT%04x E%d\n", hwcode, (swver & 0xf) + 1);

	return 0;
}
