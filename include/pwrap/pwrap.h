#ifndef _PWRAP_H
#define _PWRAP_H

struct dm_pwrap_ops {
	int (*read)(struct udevice *dev, u32 adr, u32 *rdata);
	int (*write)(struct udevice *dev, u32 adr, u32 wdata);
};

/* the host probe function */
void mtk_pwrap_init(struct udevice *dev);

/* host R/W */
u32 pwrap_readl(struct udevice *dev, u32 reg);
void pwrap_writel(struct udevice *dev, u32 val, u32 reg);

/* slave R/W */
int pwrap_read(struct udevice *dev, u32 adr, u32 *rdata);
int pwrap_write(struct udevice *dev, u32 adr, u32 wdata);

/* uclass */
int pwrap_reg_read(struct udevice *dev, u32 adr, u32 *rdata);
int pwrap_reg_write(struct udevice *dev, u32 adr, u32 wdata);

#endif /* _PWRAP_H */
