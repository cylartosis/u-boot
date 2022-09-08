// SPDX-License-Identifier: GPL-2.0

#include <common.h>
#include <command.h>
#include <dm.h>
#include <clk.h>
#include <log.h>
#include <rtc.h>
#include <asm/io.h>
#include <power/pmic.h>
#include <dm/device_compat.h>
#include <linux/delay.h>

#define RTC_TC_SEC			0x000a
#define RTC_TC_MIN			0x000c
#define RTC_TC_HOU			0x000e
#define RTC_TC_DOM			0x0010
#define RTC_TC_DOW			0x0012
#define RTC_TC_MTH			0x0014
#define RTC_TC_YEA			0x0016
#define RTC_PDN1			0x2c
#define RTC_PDN2			0x2e
#define RTC_SPAR0			0x30
#define RTC_SPAR1			0x32
#define RTC_PDN1_PWRON_TIME		(1U << 7)
#define RTC_SPAR0_PWRON_SEC_MASK	0x003f
#define RTC_SPAR1_PWRON_SEC_SHIFT	0
#define RTC_SPAR1_PWRON_MIN_MASK	0x003f
#define RTC_SPAR1_PWRON_MIN_SHIFT	0
#define RTC_SPAR1_PWRON_HOU_MASK	0x07c0
#define RTC_SPAR1_PWRON_HOU_SHIFT	6
#define RTC_SPAR1_PWRON_DOM_MASK	0xf800
#define RTC_SPAR1_PWRON_DOM_SHIFT	11
#define RTC_PDN2_PWRON_MTH_MASK		0x000f
#define RTC_PDN2_PWRON_MTH_SHIFT	0
#define RTC_PDN2_PWRON_YEA_MASK		0x7f00
#define RTC_PDN2_PWRON_YEA_SHIFT	8
#define RTC_DEFAULT_YEA			2015
#define RTC_MIN_YEAR			1968
#define RTC_DEFAULT_MTH			1
#define RTC_DEFAULT_DOM			1
#define RTC_DEFAULT_DOW			1

struct mt6328_priv {
	fdt_addr_t base;
	struct udevice *dev;
	struct clk clk;
};

static int mt6328_rtc_get(struct udevice *dev, struct rtc_time *tm)
{
	struct mt6328_priv *priv = dev_get_priv(dev);
	unsigned int now_time;

	pmic_read_u32(dev->parent, priv->base + RTC_TC_SEC, &tm->tm_sec);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_MIN, &tm->tm_min);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_HOU, &tm->tm_hour);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_DOW, &tm->tm_wday);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_DOM, &tm->tm_mday);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_MTH, &tm->tm_mon);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_YEA, &tm->tm_year);

	tm->tm_year += RTC_MIN_YEAR;
	tm->tm_mon--;

	now_time = rtc_mktime(tm);

	dev_dbg(dev, "now = %d/%d/%d (wday=%d) %d:%d:%d (%u)\n",
		tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_wday, tm->tm_hour,
		tm->tm_min, tm->tm_sec, now_time);
	
	return 0;
}

static int mt6328_rtc_set(struct udevice *dev, const struct rtc_time *tm)
{
	struct mt6328_priv *priv = dev_get_priv(dev);
	unsigned int year, mon;

	year = tm->tm_year - RTC_MIN_YEAR;
	mon = tm->tm_mon + 1;

	pmic_write_u32(dev->parent, priv->base + RTC_TC_SEC, tm->tm_sec);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_MIN, tm->tm_min);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_HOU, tm->tm_hour);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_DOW, tm->tm_wday);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_DOM, tm->tm_mday);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_MTH, mon);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_YEA, year);

	dev_dbg(dev, "set = %d/%d/%d (wday=%d) %d:%d:%d \n", year, mon,
		tm->tm_mday, tm->tm_wday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int mt6328_rtc_reset(struct udevice *dev)
{
	struct mt6328_priv *priv = dev_get_priv(dev);

	pmic_write_u32(dev->parent, priv->base + RTC_TC_SEC, 0);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_MIN, 0);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_HOU, 0);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_DOW, RTC_DEFAULT_DOW);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_DOM, RTC_DEFAULT_DOM);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_MTH, RTC_DEFAULT_MTH);
	pmic_write_u32(dev->parent, priv->base + RTC_TC_YEA, RTC_DEFAULT_YEA - RTC_MIN_YEAR);

	return 0;
}

static int mt6328_rtc_probe(struct udevice *dev)
{
	struct rtc_time *tm;
	struct mt6328_priv *priv = dev_get_priv(dev);
	unsigned int pdn1, pdn2, spar0, spar1, now_time;

	priv->base = dev_read_addr(dev);
	if (priv->base == FDT_ADDR_T_NONE) {
		dev_err(dev, "invalid address\n");
		return -EINVAL;
	}

	dev_dbg(dev, "base=%pa\n", &priv->base);

	pmic_read_u32(dev->parent, priv->base + RTC_PDN1, &pdn1);
	pmic_read_u32(dev->parent, priv->base + RTC_PDN2, &pdn2);
	pmic_read_u32(dev->parent, priv->base + RTC_SPAR0, &spar0);
	pmic_read_u32(dev->parent, priv->base + RTC_SPAR1, &spar1);

	pmic_read_u32(dev->parent, priv->base + RTC_TC_SEC , &tm->tm_sec);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_MIN , &tm->tm_min);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_HOU , &tm->tm_hour);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_DOW , &tm->tm_wday);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_DOM , &tm->tm_mday);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_MTH , &tm->tm_mon);
	pmic_read_u32(dev->parent, priv->base + RTC_TC_YEA , &tm->tm_year);

	tm->tm_year = tm->tm_year + RTC_MIN_YEAR;

	now_time = rtc_mktime(tm);

	dev_dbg(dev, "now = %d/%d/%d (wday=%d) %d:%d:%d (%u)\n",
		tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_wday, tm->tm_hour,
		tm->tm_min, tm->tm_sec, now_time);

	if(pdn1 & RTC_PDN1_PWRON_TIME) {
		tm->tm_sec = ((spar0 & RTC_SPAR0_PWRON_SEC_MASK) >> RTC_SPAR1_PWRON_SEC_SHIFT);
		tm->tm_min = ((spar1 & RTC_SPAR1_PWRON_MIN_MASK) >> RTC_SPAR1_PWRON_MIN_SHIFT);
		tm->tm_hour = ((spar1 & RTC_SPAR1_PWRON_HOU_MASK) >> RTC_SPAR1_PWRON_HOU_SHIFT);
		tm->tm_mday = ((spar1 & RTC_SPAR1_PWRON_DOM_MASK) >> RTC_SPAR1_PWRON_DOM_SHIFT);
		tm->tm_mon = ((pdn2  & RTC_PDN2_PWRON_MTH_MASK) >> RTC_PDN2_PWRON_MTH_SHIFT);
		tm->tm_year = ((pdn2  & RTC_PDN2_PWRON_YEA_MASK) >> RTC_PDN2_PWRON_YEA_SHIFT);

		tm->tm_year = tm->tm_year + RTC_MIN_YEAR;
		now_time = rtc_mktime(tm);

		dev_dbg(dev,
			"power-on time = %d/%d/%d (wday=%d) %d:%d:%d (%u)\n",
			tm->tm_year, tm->tm_mon, tm->tm_mday, tm->tm_wday,
			tm->tm_hour, tm->tm_min, tm->tm_sec, now_time);
	}

	return 0;
}

static const struct udevice_id mt6328_rtc_ids[] = {
	{ .compatible = "mtk,mt6328-rtc" }
};

static const struct rtc_ops mt6328_rtc_ops = {
	.get = mt6328_rtc_get,
	.set = mt6328_rtc_set,
	.reset = mt6328_rtc_reset,
};

U_BOOT_DRIVER(mt6328_rtc) = {
	.name = "mt6328_rtc",
	.id = UCLASS_RTC,
	.of_match = mt6328_rtc_ids,
	.ops = &mt6328_rtc_ops,
	.probe = mt6328_rtc_probe,
	.priv_auto = sizeof(struct mt6328_priv),
};
