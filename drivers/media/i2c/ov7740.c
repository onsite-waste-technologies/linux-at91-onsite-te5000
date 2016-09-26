
#include <linux/init.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-ctrls.h>

#define PID_OV7740              0x7742
#define VERSION(pid, ver)       ((pid << 8) | (ver & 0xFF))

#define REG0C       0x0C /* Register 0C */
#define   REG0C_HFLIP_IMG       0x40 /* Horizontal mirror image ON/OFF */
#define   REG0C_VFLIP_IMG       0x80 /* Vertical flip image ON/OFF */

#define REG_PID         0x0A /* Product ID Number MSB */
#define REG_VER         0x0B /* Product ID Number LSB */
#define REG_MIDH        0x1C /* Manufacturer ID byte - high */
#define REG_MIDL        0x1D /* Manufacturer ID byte - low  */

#define REG_HOUTSIZE    0x31
#define REG_VOUTSIZE    0x32
#define REG_OUTSIZE_LSB 0x34

struct ov7740_priv{
        struct v4l2_subdev       subdev;
	struct v4l2_ctrl_handler hdl;
        struct regmap           *regmap;
        struct clk              *xvclk;

        struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
};

struct ov7740_pixfmt {
	u32 code;
        enum v4l2_colorspace colorspace;
        const struct reg_sequence  *regs;
        u32 reg_num;
};

struct ov7740_framesize {
	u16 width;
	u16 height;
	const struct reg_sequence  *regs;
        u32 reg_num;
};

static const struct reg_sequence ov7740_vga[] = {
	{0x55 ,0x40},
	{0x11 ,0x02},

	{0xd5 ,0x10},
	{0x0c ,0x12},
	{0x0d ,0x34},
	{0x17 ,0x25},
	{0x18 ,0xa0},
	{0x19 ,0x03},
	{0x1a ,0xf0},
	{0x1b ,0x89},
	{0x22 ,0x03},
	{0x29 ,0x18},
	{0x2b ,0xf8},
	{0x2c ,0x01},
	{REG_HOUTSIZE ,0xa0},
	{REG_VOUTSIZE ,0xf0},
	{0x33 ,0xc4},
	{REG_OUTSIZE_LSB, 0x0},
	{0x35 ,0x05},
	//{0x36 ,0x3f},
	{0x04 ,0x60},
	{0x27 ,0x80},
	{0x3d ,0x0f},
	{0x3e ,0x80},
	{0x3f ,0x40},
	{0x40 ,0x7f},
	{0x41 ,0x6a},
	{0x42 ,0x29},
	{0x44 ,0x22},
	{0x45 ,0x41},
	{0x47 ,0x02},
	{0x49 ,0x64},
	{0x4a ,0xa1},
	{0x4b ,0x40},
	{0x4c ,0x1a},
	{0x4d ,0x50},
	{0x4e ,0x13},
	{0x64 ,0x00},
	{0x67 ,0x88},
	{0x68 ,0x1a},

	{0x14 ,0x28},
	{0x24 ,0x3c},
	{0x25 ,0x30},
	{0x26 ,0x72},
	{0x50 ,0x97},
	{0x51 ,0x1f},
	{0x52 ,0x00},
	{0x53 ,0x00},
	{0x20 ,0x00},
	{0x21 ,0xcf},
	{0x50, 0x4b},
	{0x38 ,0x14},
	{0xe9 ,0x00},
	{0x56 ,0x55},
	{0x57 ,0xff},
	{0x58 ,0xff},
	{0x59 ,0xff},
	{0x5f ,0x04},
	{0xec ,0x00},
	{0x13 ,0xff},

	//{0x80 ,0x7f},
	{0x81 ,0x3f},
	{0x82 ,0x32},
	//{0x83 ,0x01},
	{0x38 ,0x11},
	{0x84 ,0x70},
	{0x85 ,0x00},
	{0x86 ,0x03},
	{0x87 ,0x01},
	{0x88 ,0x05},
	{0x89 ,0x30},
	{0x8d ,0x30},
	{0x8f ,0x85},
	{0x93 ,0x30},
	{0x95 ,0x85},
	{0x99 ,0x30},
	{0x9b ,0x85},

	{0x9c ,0x08},
	{0x9d ,0x12},
	{0x9e ,0x23},
	{0x9f ,0x45},
	{0xa0 ,0x55},
	{0xa1 ,0x64},
	{0xa2 ,0x72},
	{0xa3 ,0x7f},
	{0xa4 ,0x8b},
	{0xa5 ,0x95},
	{0xa6 ,0xa7},
	{0xa7 ,0xb5},
	{0xa8 ,0xcb},
	{0xa9 ,0xdd},
	{0xaa ,0xec},
	{0xab ,0x1a},

	{0xce ,0x78},
	{0xcf ,0x6e},
	{0xd0 ,0x0a},
	{0xd1 ,0x0c},
	{0xd2 ,0x84},
	{0xd3 ,0x90},
	{0xd4 ,0x1e},

	{0x5a ,0x24},
	{0x5b ,0x1f},
	{0x5c ,0x88},
	{0x5d ,0x60},

	{0xac ,0x6e},
	{0xbe ,0xff},
	{0xbf ,0x00},

	{0x0f ,0x1d},
	{0x0f ,0x1f},
};

static const struct ov7740_framesize ov7740_framesizes[] = {
        { /* VGA 640x480 */
                .width          = VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.regs		= ov7740_vga,
		.reg_num        = ARRAY_SIZE(ov7740_vga),
	},
};

static struct v4l2_subdev_core_ops ov7740_subdev_core_ops = {
};

static int ov7740_mask_set(struct regmap *regmap,
			   u8  reg, u8  mask, u8  set)
{
	s32 val;
	int ret = regmap_read(regmap, reg, &val);
	if (ret < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	return regmap_write(regmap, reg, val);
}

static int ov7740_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct regmap *regmap = container_of(ctrl->handler, struct ov7740_priv, hdl)->regmap; /* priv->regmap; */
	u8 val;

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		val = ctrl->val ? REG0C_VFLIP_IMG : 0x00;
		return ov7740_mask_set(regmap, REG0C, REG0C_VFLIP_IMG, val);
	case V4L2_CID_HFLIP:
		val = ctrl->val ? REG0C_HFLIP_IMG : 0x00;
		return ov7740_mask_set(regmap, REG0C, REG0C_HFLIP_IMG, val);
	}

	return -EINVAL;
}

static const struct v4l2_ctrl_ops ov7740_ctrl_ops = {
	.s_ctrl = ov7740_s_ctrl,
};

static int ov7740_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;

	tpf->numerator = 1;
	tpf->denominator = 60;

	return 0;
}

static int ov7740_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	cp->capability = V4L2_CAP_TIMEPERFRAME;

	tpf->numerator = 1;
	tpf->denominator = 60;

	return 0;
}

static struct v4l2_subdev_video_ops ov7740_subdev_video_ops = {
	.s_parm = ov7740_s_parm,
	.g_parm = ov7740_g_parm,
};

static const struct reg_sequence ov7740_format_yuyv[] = {
	{0x12 ,0x00},
        {0x36 ,0x3f},
        {0x80, 0x7f},
        {0x83 ,0x01},
};

static const struct reg_sequence ov7740_format_bggr8[] = {
	{0x12 ,0x01},
	//{0x12 ,0x10},
        {0x36 ,0x2f},
        {0x80, 0x01},
        {0x83 ,0x04},
};

static const struct ov7740_pixfmt ov7740_formats[] = {
	{
		.code = MEDIA_BUS_FMT_YUYV8_2X8,
                .colorspace = V4L2_COLORSPACE_SRGB,
                .regs = ov7740_format_yuyv,
                .reg_num = ARRAY_SIZE(ov7740_format_yuyv),
	},
        {
                .code = MEDIA_BUS_FMT_SBGGR8_1X8,
                .colorspace = V4L2_COLORSPACE_SRGB,
                .regs = ov7740_format_bggr8,
                .reg_num = ARRAY_SIZE(ov7740_format_bggr8),
        }
};

static int ov7740_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
        u32 index = code->index;

        if (index >= ARRAY_SIZE(ov7740_formats))
                return -EINVAL;

        code->code = ov7740_formats[index].code;

        return 0;
}

static const struct ov7740_framesize *
ov7740_set_frame_size(u32 *width, u32 *height)
{
        const struct ov7740_framesize *fsize = &ov7740_framesizes[0];
        u32 frm_width = *width;
        u32 frm_height = *height;
        int i;

        for (i = 0; i < (ARRAY_SIZE(ov7740_framesizes) - 1); i++) {
                if ((fsize->width >= frm_width) &&
                    (fsize->height>= frm_height)) {
                    *width = fsize->width;
                    *height = fsize->height;

                    return fsize;
                }

                fsize++;
        }

        *width = fsize->width;
        *height = fsize->height;

        return fsize;
}


static int ov7740_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
        struct ov7740_priv *priv =
				container_of(sd, struct ov7740_priv, subdev);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
        int index = ARRAY_SIZE(ov7740_formats);
        const struct ov7740_pixfmt *pixfmt = &ov7740_formats[0];
        const struct ov7740_framesize *fsize;

        while (--index >= 0) {
		if (pixfmt->code == mf->code)
			break;

                pixfmt++;
        }

        if (index < 0)
                return -EINVAL;

        fsize = ov7740_set_frame_size(&mf->width, &mf->height);

        mf->code = ov7740_formats[index].code;
	mf->field = V4L2_FIELD_NONE;
        mf->colorspace = ov7740_formats[index].colorspace;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
                regmap_multi_reg_write(priv->regmap,
                        pixfmt->regs, pixfmt->reg_num);

                regmap_multi_reg_write(priv->regmap,
                        fsize->regs, fsize->reg_num);
        }

        return 0;
}

static int ov7740_enum_frame_interval(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->pad)
		return -EINVAL;

	if (fie->index >= 1)
		return -EINVAL;

	if ((fie->width != VGA_WIDTH) || (fie->height != VGA_HEIGHT))
		return -EINVAL;

	fie->interval.numerator = 1;
	fie->interval.denominator = 60;

	return 0;
}

static int ov7740_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->pad)
		return -EINVAL;

	if (fse->index > 0)
		return -EINVAL;

	fse->min_width = fse->max_width = VGA_WIDTH;
	fse->min_height = fse->max_height = VGA_HEIGHT;

	return 0;
}

static const struct v4l2_subdev_pad_ops ov7740_subdev_pad_ops = {
	.enum_frame_interval = ov7740_enum_frame_interval,
	.enum_frame_size = ov7740_enum_frame_size,
        .enum_mbus_code = ov7740_enum_mbus_code,
        .set_fmt = ov7740_set_fmt,
};

static struct v4l2_subdev_ops ov7740_subdev_ops = {
	.core	= &ov7740_subdev_core_ops,
	.video	= &ov7740_subdev_video_ops,
	.pad	= &ov7740_subdev_pad_ops,
};

static int ov7740_probe_dt(struct i2c_client *client,
		struct ov7740_priv *priv)
{
	/* Request the reset GPIO deasserted */
	priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
			GPIOD_OUT_LOW);
	if (!priv->resetb_gpio)
		dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
	else if (IS_ERR(priv->resetb_gpio))
		return PTR_ERR(priv->resetb_gpio);

	/* Request the power down GPIO asserted */
	priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
			GPIOD_OUT_HIGH);
	if (!priv->pwdn_gpio)
		dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
	else if (IS_ERR(priv->pwdn_gpio))
		return PTR_ERR(priv->pwdn_gpio);

	return 0;
}

static void ov7740_power(struct ov7740_priv *priv, int on)
{

        if (priv->xvclk) {
		if (on)
			clk_prepare_enable(priv->xvclk);
		else
			clk_disable_unprepare(priv->xvclk);
	}

	if (priv->pwdn_gpio)
		gpiod_direction_output(priv->pwdn_gpio, !on);

	/* We need wait for ~1ms, then access the SCCB */
	if (on)
		usleep_range(1000, 2000);
}

static int ov7740_detect(struct ov7740_priv *priv)
{
	struct regmap *regmap = priv->regmap;
        struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);
	unsigned int pid, ver, midh, midl;
	int ret;

        ov7740_power(priv, 1);

	/* Check sensor revision */
	ret = regmap_read(regmap, REG_PID, &pid);
        if (!ret)
                ret = regmap_read(regmap, REG_VER, &ver);

        switch (VERSION((u8)pid, (u8)ver)) {
	case PID_OV7740:
                regmap_read(regmap, REG_MIDH, &midh);
                regmap_read(regmap, REG_MIDL, &midl);
                dev_info(&client->dev,
		         "Product ID %0x:%0x Manufacturer ID %x:%x\n",
		         (u8)pid, (u8)ver, midh, midl);
		break;
	default:
		dev_err(&client->dev,
			"Product ID error %x:%x\n", (u8)pid, (u8)ver);
                ov7740_power(priv, 0);
		break;
	}

        return ret;
}


#define OV7740_MAX_REGISTER     0xff
static const struct regmap_config ov7740_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= OV7740_MAX_REGISTER,
};

static int ov7740_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
        struct ov7740_priv *priv;
        struct v4l2_subdev *sd;
        int ret;

        if (!i2c_check_functionality(client->adapter,
                                     I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
			"OV7740: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

        priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
                return -ENOMEM;

        priv->xvclk = devm_clk_get(&client->dev, "xvclk");
	if (IS_ERR(priv->xvclk)) {
                ret = PTR_ERR(priv->xvclk);
		dev_err(&client->dev,
			"OV7740: fail to get xvclk: %d\n", ret);
                return ret;
	}

        ret = ov7740_probe_dt(client, priv);
        if (ret)
                return ret;

        priv->regmap = devm_regmap_init_i2c(client, &ov7740_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

        sd = &priv->subdev;
        client->flags |= I2C_CLIENT_SCCB;
	v4l2_i2c_subdev_init(sd, client, &ov7740_subdev_ops);

        ret = ov7740_detect(priv);
        if (ret)
                return ret;

	v4l2_ctrl_handler_init(&priv->hdl, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov7740_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov7740_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	sd->ctrl_handler = &priv->hdl;

	ret = v4l2_ctrl_handler_setup(&priv->hdl);
	if (ret)
		return ret;

        ret = v4l2_async_register_subdev(sd);
	if (ret)
		return ret;

        return 0;
}

static int ov7740_remove(struct i2c_client *client)
{
        struct v4l2_subdev *sd = i2c_get_clientdata(client);

        v4l2_async_unregister_subdev(sd);

        return 0;
}

static const struct i2c_device_id ov7740_id[] = {
	{ "ov7740", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, ov7740_id);

static const struct of_device_id ov7740_of_match[] = {
	{.compatible = "ovti,ov7740", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov7740_of_match);


static struct i2c_driver ov7740_i2c_driver = {
	.driver = {
		.name = "ov7740",
		.of_match_table = of_match_ptr(ov7740_of_match),
	},
	.probe    = ov7740_probe,
	.remove   = ov7740_remove,
	.id_table = ov7740_id,
};
module_i2c_driver(ov7740_i2c_driver);

MODULE_DESCRIPTION("The V4L2 driver for Omni Vision 7740 sensor");
MODULE_AUTHOR("Songjun Wu <songjun.wu@atmel.com>");
MODULE_LICENSE("GPL v2");
