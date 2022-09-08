#include <clk.h>
#include <time.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <dm.h>
#include <dm/devres.h>
#include <regmap.h>
#include <dm/device_compat.h>
#include "mt6735_pwrap_hal.h"
#include <pwrap/pwrap.h>
#include <reset.h>

/* macro for wrapper status */
#define PWRAP_BASE 0x10001000
#define PWRAP_GET_WACS_RDATA(x)		(((x) >> 0) & 0x0000ffff)
#define PWRAP_GET_WACS_ARB_FSM(x)	(((x) >> 1) & 0x00000007)
#define PWRAP_GET_WACS_FSM(x)		(((x) >> 16) & 0x00000007)
#define PWRAP_GET_WACS_REQ(x)		(((x) >> 19) & 0x00000001)
#define PWRAP_STATE_SYNC_IDLE0		BIT(20)
#define PWRAP_STATE_INIT_DONE0		BIT(21)
#define PWRAP_STATE_INIT_DONE1		BIT(15)
#define PWRAP_MAN_CMD_OP_OUTS		(0x8 << 8)
#define PWRAP_MAN_CMD_OP_CSH		(0x0 << 8)
#define PWRAP_MAN_CMD_OP_CSL		(0x1 << 8)
#define PWRAP_DEW_READ_TEST_VAL		0x5aa5
/* macro for WACS FSM */
#define PWRAP_WACS_FSM_IDLE		0x00
#define PWRAP_WACS_FSM_REQ		0x02
#define PWRAP_WACS_FSM_WFDLE		0x04
#define PWRAP_WACS_FSM_WFVLDCLR		0x06
#define PWRAP_WACS_INIT_DONE		0x01
#define PWRAP_WACS_WACS_SYNC_IDLE	0x01
#define PWRAP_WACS_SYNC_BUSY		0x00

u32 pmic_readl(u32 reg)
{
	void __iomem *base;
	base = ioremap(PWRAP_BASE, 0x1000);
	return readl(base + reg);
}

void pmic_writel(u32 val, u32 reg)
{
	void __iomem *base;
	base = ioremap(PWRAP_BASE, 0x1000);
	writel(val, base + reg);
}

static u32 pwrap_get_fsm_state(struct udevice *dev)
{
	u32 val;

	val = pmic_readl(PWRAP_WACS2_RDATA);
	return PWRAP_GET_WACS_FSM(val);
}


static bool pwrap_is_fsm_vldclr(struct udevice *dev)
{
	return pwrap_get_fsm_state(dev) == PWRAP_WACS_FSM_WFVLDCLR;
}

static bool pwrap_is_fsm_idle(struct udevice *dev)
{
	u32 val = pmic_readl(PWRAP_WACS2_RDATA);

	return PWRAP_GET_WACS_FSM(val) == PWRAP_WACS_FSM_IDLE;
}

/*
 * Timeout issue sometimes caused by the last read command
 * failed because pmic wrap could not got the FSM_VLDCLR
 * in time after finishing WACS2_CMD. It made state machine
 * still on FSM_VLDCLR and timeout next time.
 * Check the status of FSM and clear the vldclr to recovery the
 * error.
 */
static inline void pwrap_leave_fsm_vldclr(struct udevice *dev)
{
	if (pwrap_is_fsm_vldclr(dev))
		pmic_writel(1, PWRAP_WACS2_VLDCLR);
}

static bool pwrap_is_sync_idle(struct udevice *dev)
{
	return pmic_readl(PWRAP_WACS2_RDATA) & PWRAP_STATE_SYNC_IDLE0;
}

static int pwrap_wait_for_state(struct udevice *dev, bool (*fp)(struct udevice *))
{
	unsigned long timeout;

	timeout = timer_get_us() + 10000;

	do {
		if (time_after(timer_get_us(), timeout))
			return fp(dev) ? 0 : -ETIMEDOUT;
		if (fp(dev))
			return 0;
	} while (1);
}

int pwrap_read(struct udevice *dev, u32 adr, u32 *rdata)
{
	int ret;
	ret = pwrap_wait_for_state(dev, pwrap_is_fsm_idle);
	if (ret) {
		pwrap_leave_fsm_vldclr(dev);
		return ret;
	}

	pmic_writel((adr >> 1) << 16, PWRAP_WACS2_CMD);

	ret = pwrap_wait_for_state(dev, pwrap_is_fsm_vldclr);
	if (ret)
		return ret;

	*rdata = PWRAP_GET_WACS_RDATA(pmic_readl(PWRAP_WACS2_RDATA));
	pmic_writel(1, PWRAP_WACS2_VLDCLR);
	return 0;
}

int pwrap_write(struct udevice *dev, u32 adr, u32 wdata)
{
	int ret;
	ret = pwrap_wait_for_state(dev, pwrap_is_fsm_idle);
	if (ret) {
		pwrap_leave_fsm_vldclr(dev);
		return ret;
	}
	pmic_writel(BIT(31) | ((adr >> 1) << 16) | wdata, PWRAP_WACS2_CMD);

	return 0;
}

static int pwrap_reset_spislave(struct udevice *dev)
{
	int ret, i;

	pmic_writel(0, PWRAP_HIPRIO_ARB_EN);
	pmic_writel(0, PWRAP_WRAP_EN);
	pmic_writel(1, PWRAP_MUX_SEL);
	pmic_writel(1, PWRAP_MAN_EN);
	pmic_writel(0, PWRAP_DIO_EN);

	pmic_writel((1 << 13) | PWRAP_MAN_CMD_OP_CSL,
			PWRAP_MAN_CMD);
	pmic_writel((1 << 13) | PWRAP_MAN_CMD_OP_OUTS,
			PWRAP_MAN_CMD);
	pmic_writel((1 << 13) | PWRAP_MAN_CMD_OP_CSH,
			PWRAP_MAN_CMD);

	for (i = 0; i < 4; i++)
		pmic_writel((1 << 13) | PWRAP_MAN_CMD_OP_OUTS,
				PWRAP_MAN_CMD);

	ret = pwrap_wait_for_state(dev, pwrap_is_sync_idle);
	if (ret) {
		pr_err("%s fail, ret=%d\n", __func__, ret);
		return ret;
	}

	pmic_writel(0, PWRAP_MAN_EN);
	pmic_writel(0, PWRAP_MUX_SEL);

	return 0;
}

int pwrap_init_sidly(struct udevice *dev)
{
	u32 rdata;
	u32 i;
	u32 pass = 0;
	signed char dly[16] = {
		-1, 0, 1, 0, 2, -1, 1, 1, 3, -1, -1, -1, 3, -1, 2, 1
	};

	for (i = 0; i < 4; i++) {
		pmic_writel(i, PWRAP_SIDLY);
		pwrap_read(dev, 0x02d6, &rdata);
		if (rdata == 0x5aa5) {
			pr_debug("[Read Test] pass, SIDLY=%x\n", i);
			pass |= 1 << i;
		}
	}

	if (dly[pass] < 0) {
		pr_err("sidly pass range 0x%x not continuous\n",
				pass);
		return -EIO;
	}

	pmic_writel(dly[pass], PWRAP_SIDLY);

	return 0;
}

u32 pmic_config_interface(struct udevice *dev, u32 RegNum, u32 val, u32 MASK, u32 SHIFT)
{
	u32 return_value = 0;
	u32 pmic_reg = 0;
	u32 rdata;

	return_value = pwrap_read(dev, RegNum, &rdata);
	pmic_reg = rdata;
	if(return_value!=0)
	{
		return return_value;
	}

	pmic_reg &= ~(MASK << SHIFT);
	pmic_reg |= (val << SHIFT);

	return_value= pwrap_write(dev, RegNum, pmic_reg);
	if(return_value!=0)
	{
		return return_value;
	}

	return return_value;
}

const static unsigned int mt6328_ovp_trim[] = {
	0x05, 0x06, 0x07, 0x07,
	0x07, 0x07, 0x07, 0x07,
	0x0d, 0x0e, 0x0f, 0x00,
	0x01, 0x02, 0x03, 0x04
};

unsigned int pmic_read_efuse(struct udevice *dev, unsigned int addr)
{
	unsigned int ret, reg_val;

	debug("pmic_6328_efuse_read started\n");

	pmic_config_interface(dev, 0x0c00, addr*2, 0x3f, 0);
	debug("pmic_config_interface(dev, 0x0c00, addr*2, 0x3f, 0);\n");
	udelay(100);
	debug("crossed the delay 100us\n");
	ret = pwrap_read(dev, 0x0c10, &reg_val);

	if (!(reg_val & 0x1))
		pmic_config_interface(dev, 0x0c10, 1, 0x1, 0);
	else
		pmic_config_interface(dev, 0x0c10, 0, 0x1, 0);

	udelay(100);
	debug("crossed the delay 100us\n");

	do
		ret = pwrap_read(dev, 0x0c1a, &reg_val);
	while(reg_val && 0x1 ==1);

	debug("before the delay 1000us\n");
	udelay(1000);
	debug("crossed the delay 1000us\n");

	ret = pwrap_read(dev, 0x0c18, &reg_val);

	return (reg_val & 0xffff);

}

u32 efuse_data[0x20]={0};

void pmic_6328_efuse_management(struct udevice *dev)
{
	int i=0;
	int is_efuse_trimed=0;
	u16 status = 0;
	u32 data32, data32_chk,data32_448_org,data32_472_org, data32_448,data32_472, trim;

	debug("pmic_6328_efuse_management started\n");

	debug("before pwrap_read\n");
	pwrap_read(dev, 0xc5c, &trim);
	is_efuse_trimed = (((trim) >> 15) & 0x0001);
	debug("after pwrap_read\n");

	if(is_efuse_trimed == 1) {
		//get efuse data
		//turn on efuse clock
		pmic_config_interface(dev, 0x0278, 0x00, 0x1, 6);
		pmic_config_interface(dev, 0x024e, 0x00, 0x1, 2);
		pmic_config_interface(dev, 0x0c16, 0x01, 0x1, 0);

		for(i=0;i<=0x1f;i++)
		{
			efuse_data[i]=pmic_read_efuse(dev, i);
		}

		// dump efuse data for check
		for(i=0x0;i<=0x1f;i++)
			debug("[6328]e-data[0x%x]=0x%x\n", i, efuse_data[i]);


		//debug("Before apply pmic efuse\n");
		//pmic_6328_efuse_check();

		//------------------------------------------
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>0) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>1) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>2) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>3) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>4) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>5) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>6) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>7) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>8) & 0x0001), 0x1 ,12);
		pmic_config_interface(dev, 0x0434, ((efuse_data[0] >>9) & 0x0001), 0x1 ,13);
		pmic_config_interface(dev, 0x0438, ((efuse_data[0] >>10) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x0438, ((efuse_data[0] >>11) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x0438, ((efuse_data[0] >>12) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x0438, ((efuse_data[0] >>13) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0438, ((efuse_data[0] >>14) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0464, ((efuse_data[0] >>15) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x0464, ((efuse_data[1] >>0) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x0464, ((efuse_data[1] >>1) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x0464, ((efuse_data[1] >>2) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0464, ((efuse_data[1] >>3) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0464, ((efuse_data[1] >>4) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>5) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>6) & 0x0001), 0x1 ,12);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>7) & 0x0001), 0x1 ,13);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>8) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>9) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>10) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>11) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>12) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0438, ((efuse_data[1] >>13) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x046E, ((efuse_data[1] >>14) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x046E, ((efuse_data[1] >>15) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>0) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>1) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>2) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>3) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>4) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>5) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>6) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>7) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>8) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x046E, ((efuse_data[2] >>9) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>10) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>11) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>12) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>13) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>14) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0444, ((efuse_data[2] >>15) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>0) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>1) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>2) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>3) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>4) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0444, ((efuse_data[3] >>5) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>6) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>7) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>8) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>9) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>10) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>11) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>12) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>13) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>14) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0458, ((efuse_data[3] >>15) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0458, ((efuse_data[4] >>0) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0458, ((efuse_data[4] >>1) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>2) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>3) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>4) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>5) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>6) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>7) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>8) & 0x0001), 0x1 ,0);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>9) & 0x0001), 0x1 ,1);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>10) & 0x0001), 0x1 ,2);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>11) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>12) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x044E, ((efuse_data[4] >>13) & 0x0001), 0x1 ,5);
		pmic_config_interface(dev, 0x0A52, ((efuse_data[4] >>14) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A52, ((efuse_data[4] >>15) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A52, ((efuse_data[5] >>0) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A52, ((efuse_data[5] >>1) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A56, ((efuse_data[5] >>2) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A56, ((efuse_data[5] >>3) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A56, ((efuse_data[5] >>4) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A56, ((efuse_data[5] >>5) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A58, ((efuse_data[5] >>6) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A58, ((efuse_data[5] >>7) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A58, ((efuse_data[5] >>8) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A58, ((efuse_data[5] >>9) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A7C, ((efuse_data[5] >>10) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A7C, ((efuse_data[5] >>11) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A7C, ((efuse_data[5] >>12) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A7C, ((efuse_data[5] >>13) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A7E, ((efuse_data[5] >>14) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A7E, ((efuse_data[5] >>15) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A7E, ((efuse_data[6] >>0) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A7E, ((efuse_data[6] >>1) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A60, ((efuse_data[6] >>2) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A60, ((efuse_data[6] >>3) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A60, ((efuse_data[6] >>4) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A60, ((efuse_data[6] >>5) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A62, ((efuse_data[6] >>6) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A62, ((efuse_data[6] >>7) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A62, ((efuse_data[6] >>8) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A62, ((efuse_data[6] >>9) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A66, ((efuse_data[6] >>10) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A66, ((efuse_data[6] >>11) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A66, ((efuse_data[6] >>12) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A66, ((efuse_data[6] >>13) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A64, ((efuse_data[6] >>14) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A64, ((efuse_data[6] >>15) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A64, ((efuse_data[7] >>0) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A64, ((efuse_data[7] >>1) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A72, ((efuse_data[7] >>2) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A72, ((efuse_data[7] >>3) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A72, ((efuse_data[7] >>4) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A72, ((efuse_data[7] >>5) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A84, ((efuse_data[7] >>6) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A84, ((efuse_data[7] >>7) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A84, ((efuse_data[7] >>8) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A84, ((efuse_data[7] >>9) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A7A, ((efuse_data[7] >>10) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A7A, ((efuse_data[7] >>11) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A7A, ((efuse_data[7] >>12) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A7A, ((efuse_data[7] >>13) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A5C, ((efuse_data[7] >>14) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A5C, ((efuse_data[7] >>15) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A5C, ((efuse_data[8] >>0) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A5C, ((efuse_data[8] >>1) & 0x0001), 0x1 ,12);
		pmic_config_interface(dev, 0x0A6A, ((efuse_data[8] >>2) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A6A, ((efuse_data[8] >>3) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A6A, ((efuse_data[8] >>4) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A6A, ((efuse_data[8] >>5) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x0A6C, ((efuse_data[8] >>6) & 0x0001), 0x1 ,8);
		pmic_config_interface(dev, 0x0A6C, ((efuse_data[8] >>7) & 0x0001), 0x1 ,9);
		pmic_config_interface(dev, 0x0A6C, ((efuse_data[8] >>8) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x0A6C, ((efuse_data[8] >>9) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x043E, ((efuse_data[8] >>10) & 0x0001), 0x1 ,10);
		pmic_config_interface(dev, 0x043E, ((efuse_data[8] >>11) & 0x0001), 0x1 ,11);
		pmic_config_interface(dev, 0x043E, ((efuse_data[8] >>12) & 0x0001), 0x1 ,12);

		pmic_config_interface(dev, 0x0470, ((efuse_data[12] >>7) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0470, ((efuse_data[12] >>8) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x046C, ((efuse_data[12] >>9) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x046C, ((efuse_data[12] >>10) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0466, ((efuse_data[12] >>11) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0466, ((efuse_data[12] >>12) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x0442, ((efuse_data[12] >>13) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0442, ((efuse_data[12] >>14) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x045A, ((efuse_data[12] >>15) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x045A, ((efuse_data[13] >>0) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x0456, ((efuse_data[13] >>1) & 0x0001), 0x1 ,3);
		pmic_config_interface(dev, 0x0456, ((efuse_data[13] >>2) & 0x0001), 0x1 ,4);
		pmic_config_interface(dev, 0x0450, ((efuse_data[13] >>3) & 0x0001), 0x1 ,6);
		pmic_config_interface(dev, 0x0450, ((efuse_data[13] >>4) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x044C, ((efuse_data[13] >>5) & 0x0001), 0x1 ,7);
		pmic_config_interface(dev, 0x044C, ((efuse_data[13] >>6) & 0x0001), 0x1 ,8);

		debug("before pmic_read_efuse\n");
		efuse_data[0x11]=pmic_read_efuse(dev, 0x11);
		debug("after pmic_read_efuse\n");

		/* check bit 784 if equal to zero */
		if ((efuse_data[0x11] & 0x1) == 0)
		{
			/* read ovp original trim value */
			status = pwrap_read(dev, 0x0f80, &data32);
			/* remap to new value and write back */
			pmic_config_interface(dev, 0x0f80, mt6328_ovp_trim[data32], 0xf, 0);
			/* read ovp original trim value */
			status = pwrap_read(dev, 0x0f80, &data32_chk);
			debug("[pmic_6328_efuse_management]4.27 org_ovp_trim:0x%x ovp_trim[0x%x]:0x%x new_ovp_trim:0x%x\r\n", data32, 0x0f80, mt6328_ovp_trim[data32], data32_chk);
		}
		else
			debug("[pmic_6328_efuse_management]4.27 efuse_data[0x11]:0x%x\r\n", efuse_data[0x11]);

		if ((efuse_data[0x11] & 0x2) == 0x2)
		{
			/* read 448 value */
			status = pwrap_read(dev, 0x0448, &data32_448_org);
			pmic_config_interface(dev, 0x0448, 0, 0x1, 0);
			status = pwrap_read(dev, 0x0448, &data32_448);
			/* read 472 value */
			status = pwrap_read(dev, 0x0472, &data32_472_org);
			pmic_config_interface(dev, 0x0472, 0, 0x1, 0);
			status = pwrap_read(dev, 0x0472, &data32_472);
			debug("[pmic_6328_efuse_management]4.27 data32_448_org:0x%x data32_472_org:0x%x\r\n", (data32_448_org & 0xffff), (data32_472_org &0xffff));
			debug("[pmic_6328_efuse_management]4.27 data32_448:0x%x data32_472:0x%x\r\n", (data32_448 & 0xffff), (data32_472 & 0xffff));

		} else
			debug("[pmic_6328_efuse_management]4.27 efuse_data[0x11][448:472]:0x%x\r\n", efuse_data[0x11]);

		//------------------------------------------

		//debug("After apply pmic efuse\n");
		//pmic_6328_efuse_check();

		//turn off efuse clock
		pmic_config_interface(dev, 0x0278, 0x01, 0x1, 2);
		pmic_config_interface(dev, 0x024e, 0x01, 0x1, 6);
		pmic_config_interface(dev, 0x0616, 0x68, 0x7f, 0);
		pmic_config_interface(dev, 0x0618, 0x68, 0x7f, 0);
		pmic_config_interface(dev, 0x4b4, 0x68, 0xFFFF, 0x0);
		pmic_config_interface(dev, 0xA88, 0x68, 0xFFFF, 0x0);
	}
}
void mtk_pwrap_init(struct udevice *dev)
{
	writel(0x80, INFRA_GLOBALCON_RST0);
	writel(0x80, INFRA_GLOBALCON_RST1);
	writel(0x70000, 0x10000088);
	pmic_writel(3, PWRAP_DCM_EN);
	pmic_writel(0, PWRAP_DCM_DBC_PRD);
	pwrap_reset_spislave(dev);
	pmic_writel(1, PWRAP_WRAP_EN);
	pmic_writel(0x3fff, PWRAP_HIPRIO_ARB_EN);
	pmic_writel(1, PWRAP_WACS2_EN);
	pmic_writel(0x88, PWRAP_RDDMY);
	pwrap_write(dev, 0x02ee, 0x8);
	pmic_writel(0x0, PWRAP_CSHEXT_READ);
	pmic_writel(0x33, PWRAP_CSHEXT_WRITE);
	pmic_writel(0x0, PWRAP_CSLEXT_START);
	pmic_writel(0x0, PWRAP_CSLEXT_END);
	pwrap_init_sidly(dev);
	pwrap_write(dev, 0x02d4, 1);
	debug("pwrap_write(dev, 0x02d4, 1)");
	pmic_writel(1, PWRAP_DIO_EN);
	debug("pmic_writel(1, PWRAP_DIO_EN)");
	pmic_writel(0x1, PWRAP_WACS0_EN);
	debug("pmic_writel(0x1, PWRAP_WACS0_EN)");
	pmic_writel(0x1, PWRAP_WACS1_EN);
	debug("pmic_writel(0x1, PWRAP_WACS1_EN)");
	pmic_writel(0x1, PWRAP_WACS2_EN);
	debug("pmic_writel(0x1, PWRAP_WACS2_EN)");
	pmic_writel(0x5, PWRAP_STAUPD_PRD);
	debug("pmic_writel(0x5, PWRAP_STAUPD_PRD)");
	pmic_writel(0xf, PWRAP_WDT_UNIT);
	debug("pmic_writel(0xf, PWRAP_WDT_UNIT)");
	pmic_writel(0xfffffbff, PWRAP_WDT_SRC_EN);
	debug("pmic_writel(0xfffffbff, PWRAP_WDT_SRC_EN)");
	pmic_writel(0x1, PWRAP_TIMER_EN);
	debug("pmic_writel(0x1, PWRAP_TIMER_EN)");
	pmic_writel(0xfffffbfb, PWRAP_INT_EN);
	debug("pmic_writel(0xfffffbfb, PWRAP_INT_EN)");
	pmic_writel(1, PWRAP_INIT_DONE2);
	debug("pmic_writel(1, PWRAP_INIT_DONE2)");
	pmic_writel(1, PWRAP_INIT_DONE0);
	debug("pmic_writel(1, PWRAP_INIT_DONE0)");
	pmic_writel(1, PWRAP_INIT_DONE1);
	debug("pmic_writel(1, PWRAP_INIT_DONE1)");
	pmic_6328_efuse_management(dev);
}
