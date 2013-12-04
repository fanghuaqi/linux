/*
 * Copyright (C) 2013 Synopsys, Inc. (www.synopsys.com)
 *
 *   Mischa Jonker <mjonker@synopsys.com>
 *
 *   arcpgu.c
 *
 *  Simple fb driver with hardcoded 640x480x24 resolution
 *  can be used with nSIM OSCI model
 *
 *  based on: linux/drivers/video/skeletonfb.c
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License v2. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#define DEBUG
#include <linux/platform_device.h>
#undef DEBUG
#include <linux/dma-mapping.h>
#include <linux/of.h>


/* Register definitions */

#define PGU_CTRL_CONT_MASK      (0x1)
#define PGU_CTRL_ENABLE_MASK    (0x2)
#define PGU_CTRL_FORMAT_MASK    (0x4)
#define PGU_CTRL_VS_POL_MASK    (0x8)
#define PGU_CTRL_HS_POL_MASK    (0x10)
#define PGU_CTRL_DE_POL_MASK    (0x20)
#define PGU_CTRL_CLK_POL_MASK   (0x40)
#define PGU_CTRL_CLK_HIGH_MASK  (0xff0000)
#define PGU_CTRL_CLK_DIV_MASK   (0xff000000)
#define PGU_CTRL_CLK_OFFSET	16

#define GET_PGU_CTRL_CLK_HIGH_VAL(x) ((x & PGU_CTRL_CLK_HIGH_MASK) >> 16)
#define SET_PGU_CTRL_CLK_HIGH_VAL(x, y) ((x & PGU_CTRL_CLK_HIGH_MASK) |\
		((y << 16) & PGU_CTRL_CLK_DIV_MASK))

#define PGU_STAT_ERR_MASK	(0x1)
#define PGU_STAT_BUSY_MASK	(0x2)
#define PGU_STAT_PENDING_MASK   (0x4)
#define PGU_STAT_START_MASK	(0x18)

#define PGU_FMT_Y_RES_MASK	0xfff
#define PGU_FMT_X_RES_MASK	0xfff0000
#define PGU_FMT_X_SHIFT         16

#define ENCODE_PGU_FMT(x, y) (((x * 0x10000) & PGU_FMT_X_RES_MASK)\
	| (y & PGU_FMT_Y_RES_MASK))

#define PGU_HSYNC_ACT_MASK	0xfff0000
#define PGU_HSYNC_DEACT_MASK	0xfff
#define ENCODE_PGU_HSYNC_TIM(x, y) (((x * 0x10000) &\
	PGU_HSYNC_ACT_MASK) | (y & PGU_HSYNC_DEACT_MASK))

#define PGU_VSYNC_ACT_MASK	0xfff0000
#define PGU_VSYNC_DEACT_MASK	0xfff
#define ENCODE_PGU_VSYNC_TIM(x, y) (((x * 0x10000) &\
	PGU_VSYNC_ACT_MASK) | (y & PGU_VSYNC_DEACT_MASK))

#define PGU_FRAME_HACT_MASK 0xfff0000
#define PGU_FRAME_VACT_MASK 0xfff
#define ENCODE_PGU_FRAME_TIM(x, y) (((x * 0x10000) & PGU_FRAME_HACT_MASK)\
	| (y & PGU_FRAME_VACT_MASK))

#define PGU_HSYNC_OFFSET	0x18
#define PGU_VSYNC_OFFSET	0x20

/*---------------------------------------------------------------------------*/
/* arc_pgu regs*/
struct arc_pgu_regs {
	uint32_t  ctrl;
	uint32_t  stat;
	uint32_t  padding1[2];
	uint32_t  fmt;
	uint32_t  hsync;
	uint32_t  vsync;
	uint32_t  frame;
	uint32_t  padding2[8];
	uint32_t  base0;
	uint32_t  base1;
	uint32_t  base2;
	uint32_t  padding3[1];
	uint32_t  stride;
	uint32_t  padding4[11];
	uint32_t  start_clr;
	uint32_t  start_set;
	uint32_t  padding5[206];
	uint32_t  int_en_clr;
	uint32_t  int_en_set;
	uint32_t  int_en;
	uint32_t  padding6[5];
	uint32_t  int_stat_clr;
	uint32_t  int_stat_set;
	uint32_t  int_stat;
	uint32_t  padding7[4];
	uint32_t  module_id;
};

/* display information */
struct known_displays {
	uint32_t xres;
	uint32_t yres;
	uint32_t bpp;
	unsigned char display_name[256];
	uint32_t control;
	uint32_t clkcontrol;
	uint32_t left_margin;
	uint32_t upper_margin;
	uint32_t hsync_start;
	uint32_t hsync_end;
	uint32_t vsync_start;
	uint32_t vsync_end;
	/* this value decides whether needs to dive clk */
	uint32_t max_freq;
};

/* parameters for arc pgu */
struct arcpgu_par {
	struct arc_pgu_regs __iomem *regs;
	void *fb;		/* frame buffer's virtual  address */
	dma_addr_t fb_phys;	/* frame buffer's physical address */
	uint32_t fb_size;	/* frame buffer's size */
	struct known_displays *display;	/* current display info */
/* the following comes from arc_pgu2_par */
	int line_length;
	int cmap_len;
	uint32_t main_mode;
	uint32_t overlay_mode;
	int num_rgbbufs;
	int rgb_bufno;
	int main_is_fb;
};


/* screen information */
struct known_displays dw_displays[] = {
	{ /* ARCPGU_DISPTYPE:0 Demo display */
		640,		/* xres */
		480,		/* yres */
		16,		/* bpp  */
		"Philips 640x480 color(16bpp) LCD monitor",
		/* mode control */
		(PGU_CTRL_VS_POL_MASK | PGU_CTRL_HS_POL_MASK),
		(0x0100),	/* clkcontrol */
		(10),		/* left margin */
		(10),		/* upper margin*/
		(0x0),		/* hsync start */
		(0x6f),		/* hsync end */
		(0x0),		/* vsync start */
		(0xb),		/* vsync  end */
		(15000000)	/* max_freq */
	},
	{ /* ARCPGU_DISPTYPE:1 Hitachi 640x480 TFT panel */
		640,		/* xres */
		480,		/* yres */
		16,			/* bpp  */
		"Hitachi TX14D11VM1CBA 640x480 colour(16bpp) LCD",
		(0),		/* mode control */
		(0x0102),	/* clkcontrol */
		(10),		/* left margin */
		(10),		/* upper margin*/
		(0),		/* hsync start */
		(0x04220180),	/* hsync end */
		(0),		/* vsync start */
		(0x010500fc),	/* vsync end */
		(12000000)	/* max_freq */
	},
#if 0
	{ /* ARCPGU_DISPTYPE:2 ADV7511 Transmitter */
		1024,		/* xres */
		768,		/* yres */
		16,			/* bpp  */
		"Analog Device ADV7511 HDMI Transmitter",
		(0),		/* mode control */
		(0x0102),	/* clkcontrol */
		(319),		/* left margin */
		(37),		/* upper margin*/
		(24),		/* hsync start */
		(159),		/* hsync end */
		(2),		/* vsync start */
		(8),		/* vsync end */
		(165000000)	/* max_freq */
	}
#endif
	{ /* ARCPGU_DISPTYPE:2 ADV7511 Transmitter */
		640,		/* xres */
		480,		/* yres */
		16,			/* bpp  */
		"Analog Device ADV7511 HDMI Transmitter",
		(0),		/* mode control */
		(0x0102),	/* clkcontrol */
		(159),		/* left margin */
		(44),		/* upper margin*/
		(15),		/* hsync start */
		(111),		/* hsync end */
		(9),		/* vsync start */
		(11),		/* vsync end */
		(25200000)	/* max_freq */
	}

};

/* the screen parameters that can be modified by the user */
static struct fb_var_screeninfo arcpgufb_var = {
	.xres =			640,
	.yres =			480,
	.xoffset =		0,
	.yoffset =		0,
	.xres_virtual =		640,
	.yres_virtual =		480,
/* RGB888 */
/*
	.bits_per_pixel =	24,
	.red =			{.offset = 16, .length = 8},
	.green =		{.offset = 8, .length = 8},
	.blue =			{.offset = 0, .length = 8},
*/
/* RGB565 */
	.bits_per_pixel =	16,
	.red =			{.offset = 11, .length = 5},
	.green =		{.offset = 5, .length = 6},
	.blue =			{.offset = 0, .length = 5},
};

/* the screen parameters that is fixed */
static struct fb_fix_screeninfo arcpgufb_fix = {
	.id =		"arcpgufb",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	0,
	.ypanstep =	0,
	.ywrapstep =	0,
	.accel =	FB_ACCEL_NONE,	/* no hardware acceleration */
};


static void arcpgufb_disable(struct fb_info *info)
{
	unsigned int val;
	struct arcpgu_par *par = info->par;
	
	val = ioread32(&par->regs->ctrl);
	val &= ~PGU_CTRL_ENABLE_MASK;
	iowrite32(val, &par->regs->ctrl);
	
	while (ioread32(&par->regs->stat) & PGU_STAT_BUSY_MASK)
		;
}

static void arcpgufb_enable(struct fb_info *info)
{
	unsigned int val;
	struct arcpgu_par *par = info->par;

	val = ioread32(&par->regs->ctrl);
	val |= PGU_CTRL_ENABLE_MASK;
	
	iowrite32(val, &par->regs->ctrl);
	
	while (ioread32(&par->regs->stat) & PGU_STAT_BUSY_MASK)
		;
}


static int arcpgufb_check_var(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	/* always return OK */
	*var = info->var;
	return 0;
}

static int arcpgufb_set_par(struct fb_info *info)
{
	
	int i;
	uint32_t xfmt, yfmt, act, deact;

	struct arcpgu_par *par = info->par;

	arcpgufb_disable(info);
		/* Initialize controller */
	xfmt = info->var.xres + info->var.left_margin;
	yfmt = info->var.yres + info->var.upper_margin;
		
	iowrite32(ENCODE_PGU_FMT(xfmt, yfmt), &par->regs->fmt);
	dev_info(info->dev, "FMT:%x", ioread32(&par->regs->fmt));
	
	act = info->var.right_margin - info->var.hsync_len;
	deact = info->var.right_margin;
	iowrite32(ENCODE_PGU_HSYNC_TIM(act, deact), &par->regs->hsync);

	act = info->var.lower_margin - info->var.vsync_len;
	deact = info->var.lower_margin;
	iowrite32(ENCODE_PGU_VSYNC_TIM(act, deact), &par->regs->vsync);

	iowrite32(ENCODE_PGU_FRAME_TIM(info->var.left_margin,
			info->var.upper_margin), &par->regs->frame);

	iowrite32(par->fb_phys, &par->regs->base0);

	if (par->num_rgbbufs > 1) {
		iowrite32(par->fb_phys + (par->fb_size / par->num_rgbbufs),
		&par->regs->base1); /* base1, double buffer */
	}	
	if (par->num_rgbbufs > 2) {
		iowrite32(par->fb_phys + 2 * (par->fb_size / par->num_rgbbufs),
		&par->regs->base2); /* base2, tripple buffer */
	}

	iowrite32(0, &par->regs->stride);	/* stride */

	/* change ctrl according to  info->par, no fix mode */
	/* RGB:555, continugous mode */
	iowrite32(0x01000061, &par->regs->ctrl);
	arcpgufb_enable(info);

	/* start dma transfer for frame buffer 0  */
	iowrite32(1, &par->regs->start_set);
	dev_dbg(info->dev, "CTRL:%x", ioread32(&par->regs->ctrl));
	return 0;
}

/* This function is required for correct operation of frame buffer console */
static int arcpgufb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
	uint32_t v;

	if (regno >= 16)
		return -EINVAL;
	
	/* rgb 565 */
	red = red	>> (16 - info->var.red.length);
	green = green	>> (16 - info->var.green.length);
	blue = blue	>> (16 - info->var.blue.length);
	transp = transp	>> (16 - info->var.transp.length);

	v = (red    << info->var.red.offset)   |
	    (green  << info->var.green.offset) |
	    (blue   << info->var.blue.offset)  |
	    (transp << info->var.transp.offset);

	((uint32_t *)(info->pseudo_palette))[regno] = v;

	return 0;
}

static struct fb_ops arcpgufb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= arcpgufb_check_var,
	.fb_set_par	= arcpgufb_set_par,
	.fb_setcolreg	= arcpgufb_setcolreg,
	.fb_fillrect	= cfb_fillrect,
	.fb_copyarea	= cfb_copyarea,
	.fb_imageblit	= cfb_imageblit,
};

/* ------------------------------------------------------------------------- */

static int arcpgufb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct arcpgu_par *par;
	struct device *device = &pdev->dev;
	struct resource *res;
	int retval;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(device, "no IO memory defined\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct arcpgu_par), device);
	if (!info) {
		dev_err(device, "could not allocate framebuffer\n");
		return -ENOMEM;
	}

	par = info->par;
/*	get config display info, here use retval as index */
	retval = CONFIG_ARCPGU_DISPTYPE;
/* check retval ?*/
	par->display = &dw_displays[retval];

	/* var setting according to display */
	arcpgufb_var.xres = par->display->xres;
	arcpgufb_var.xres_virtual = par->display->xres;
	arcpgufb_var.yres = par->display->yres;
	arcpgufb_var.yres_virtual = par->display->yres * CONFIG_ARCPGU_RGBBUFS;
	arcpgufb_var.bits_per_pixel = par->display->bpp;
	arcpgufb_var.left_margin = par->display->left_margin;
	arcpgufb_var.upper_margin = par->display->upper_margin;
	arcpgufb_var.hsync_len = par->display->hsync_end -
			par->display->hsync_start;
	arcpgufb_var.right_margin = par->display->hsync_end;
	arcpgufb_var.vsync_len = par->display->vsync_end -
			par->display->vsync_start;
	arcpgufb_var.lower_margin = par->display->vsync_end;

	/* only works for 8/16 bpp */
	par->num_rgbbufs = CONFIG_ARCPGU_RGBBUFS;
	par->line_length = par->display->xres
		* par->display->bpp / 8;
	par->main_mode = 1;
	par->overlay_mode = 1;
	par->rgb_bufno = 0;
	par->main_is_fb = 1;
	par->cmap_len = (par->display->bpp == 8) ? 256 : 16;


	par->regs = devm_request_and_ioremap(device, res);

	dev_info(device, "arc_pgu ID# 0x%x, using the: %s\n",
		ioread32(&par->regs->module_id), par->display->display_name);

	par->fb_size = CONFIG_ARCPGU_RGBBUFS * arcpgufb_var.xres *
		arcpgufb_var.yres * ((arcpgufb_var.bits_per_pixel + 7) >> 3);
	dev_dbg(device, "fb size:%x\n", par->fb_size);
	par->fb = dma_alloc_coherent(device, PAGE_ALIGN(par->fb_size),
				     &par->fb_phys, GFP_KERNEL);
	if (!par->fb) {
		retval = -ENOMEM;
		goto out2;
	}
	
	dev_dbg(device, "framebuffer at: 0x%x (logical), 0x%x (physical)\n",
		par->fb, par->fb_phys);

	/* encode_fix */
	arcpgufb_fix.smem_start = par->fb_phys;
	arcpgufb_fix.smem_len = par->fb_size;
	arcpgufb_fix.mmio_start = (uint32_t)par->regs;
	arcpgufb_fix.mmio_len = sizeof(struct arc_pgu_regs);
	arcpgufb_fix.visual = (arcpgufb_var.bits_per_pixel == 8) ?
		FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;
	arcpgufb_fix.line_length = par->line_length;

/* info setting */
	info->pseudo_palette = devm_kzalloc(device, sizeof(uint32_t) * 16,
					    GFP_KERNEL);
	if (!info->pseudo_palette) {
		retval = -ENOMEM;
		goto out;
	}
	memset(info->pseudo_palette, 0, sizeof(uint32_t) * 16);

	info->fbops = &arcpgufb_ops;
	info->flags = FBINFO_DEFAULT;

	if (fb_alloc_cmap(&info->cmap, 256, 0)) {
		dev_err(device, "could not allocate cmap\n");
		retval = -ENOMEM;
		goto out;
	}

	info->screen_base = par->fb;
	
	info->fix = arcpgufb_fix;
	info->var = arcpgufb_var;
	
	if (register_framebuffer(info) < 0) {
		retval = -EINVAL;
		goto out;
	}

	platform_set_drvdata(pdev, info);
	return 0;

out:
	dma_free_coherent(device, PAGE_ALIGN(par->fb_size),
			  par->fb, par->fb_size);
out2:
	fb_dealloc_cmap(&info->cmap);
	return retval;
}


static int arcpgufb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	struct arcpgu_par *par = info->par;

	if (info) {
		unregister_framebuffer(info);
		dma_free_coherent(&pdev->dev, PAGE_ALIGN(par->fb_size),
				  par->fb, par->fb_size);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id arcpgufb_match[] = {
	{ .compatible = "snps,arcpgufb" },
	{},
};
#endif

static struct platform_driver arcpgufb_driver = {
	.probe = arcpgufb_probe,
	.remove = arcpgufb_remove,
	.driver = {
		.name = "arcpgufb",
		.of_match_table = of_match_ptr(arcpgufb_match),
	},
};

static int __init arcpgufb_init(void)
{
	return platform_driver_register(&arcpgufb_driver);
}

static void __exit arcpgufb_exit(void)
{
	platform_driver_unregister(&arcpgufb_driver);
}

/* ------------------------------------------------------------------------- */

module_init(arcpgufb_init);
module_exit(arcpgufb_exit);

MODULE_LICENSE("GPL");
