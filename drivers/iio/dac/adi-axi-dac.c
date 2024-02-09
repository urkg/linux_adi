// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic AXI DAC IP core
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_dac_ip
 *
 * Copyright 2016-2024 Analog Devices Inc.
 */

#include "linux/array_size.h"
#include "linux/limits.h"
#include "linux/units.h"
#include "vdso/bits.h"
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>

/*
 * Register definitions:
 *   https://wiki.analog.com/resources/fpga/docs/axi_dac_ip#register_map
 */

/* Base controls */
#define AXI_DAC_REG_CONFIG		0x0c
#define	   AXI_DAC_DISABLE		BIT(6)

 /* DAC controls */
#define AXI_DAC_REG_RSTN		0x0040
#define   AXI_DAC_RSTN_CE_N		BIT(2)
#define   AXI_DAC_RSTN_MMCM_RSTN	BIT(1)
#define   AXI_DAC_RSTN_RSTN		BIT(0)
#define AXI_DAC_REG_CNTRL_1		0x0044
#define   AXI_DAC_SYNC			BIT(0)
/* DAC Channel controls */
#define AXI_DAC_REG_CHAN_CNTRL_1(c)	(0x0400 + (c) * 0x40)
#define AXI_DAC_REG_CHAN_CNTRL_3(c)	(0x0400 + (c) * 0x40)
#define   AXI_DAC_SCALE_SIGN		BIT(15)
#define   AXI_DAC_SCALE_INT		BIT(14)
#define   AXI_DAC_SCALE_INT_NEG		GENMASK(15, 14)
#define   AXI_DAC_SCALE_FRAC		GENMASK(13, 0)
#define AXI_DAC_REG_CHAN_CNTRL_2(c)	(0x0404 + (c) * 0x40)
#define AXI_DAC_REG_CHAN_CNTRL_4(c)	(0x040c + (c) * 0x40)
#define   AXI_DAC_PHASE			GENMASK(31, 16)
#define AXI_DAC_REG_CHAN_CNTRL_7(c)	(0x0418 + (c) * 0x40)
#define   AXI_DAC_DATA_SEL		GENMASK(3, 0)

/* 360 degrees is rad */
#define AXI_DAC_2_PI_MEGA		6283190
enum {
	AXI_DAC_DATA_INTERNAL_TONE,
	AXI_DAC_DATA_DMA = 2,
};

struct axi_dac_state {
	struct regmap *regmap;
	struct device *dev;
	struct mutex lock;
	u64 dac_clk;
	u32 reg_config;
};

static int axi_dac_enable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);
	int ret;

	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_RSTN,
			      AXI_DAC_RSTN_MMCM_RSTN);
	if (ret)
		return ret;

	fsleep(10);
	return regmap_set_bits(st->regmap, AXI_DAC_REG_RSTN,
			       AXI_DAC_RSTN_RSTN | AXI_DAC_RSTN_MMCM_RSTN);
}

static void axi_dac_disable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	regmap_write(st->regmap, AXI_DAC_REG_RSTN, 0);
}

static struct iio_buffer *axi_dac_request_buffer(struct iio_backend *back,
						 struct iio_dev *indio_dev)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);
	struct iio_buffer *buffer;
	const char *dma_name;
	int ret;

	if (device_property_read_string(st->dev, "dma-names", &dma_name))
		dma_name = "tx";

	buffer = iio_dmaengine_buffer_alloc(st->dev, dma_name);
	if (IS_ERR(buffer)) {
		dev_err(st->dev, "Could not get DMA buffer, %ld\n",
			PTR_ERR(buffer));
		return ERR_CAST(buffer);
	}

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_buffer_set_dir(buffer, IIO_BUFFER_DIRECTION_OUT);

	ret = iio_device_attach_buffer(indio_dev, buffer);
	if (ret)
		return ERR_PTR(ret);

	return buffer;
}

static void axi_dac_free_buffer(struct iio_backend *back,
				struct iio_buffer *buffer)
{
	iio_dmaengine_buffer_free(buffer);
}

static ssize_t axi_dac_frequency_read(struct axi_dac_state *st,
				      uintptr_t private, char *buf)
{
	dev_info(st->dev, "Get frequency, tone=%lu\n", private + 1);
	return 0;
}

static ssize_t axi_dac_scale_read(struct axi_dac_state *st,
				  const struct iio_chan_spec *chan,
				  uintptr_t private, char *buf)
{
	int vals[2] = {0};
	u8 sign, integer;
	u32 reg, __val;
	u16 frac;
	int ret;

	dev_info(st->dev, "Get scale, tone=%lu\n", private == AXI_DAC_SCALE_TONE_1 ? 1UL : 2UL);

	if (private == AXI_DAC_SCALE_TONE_1)
		reg = AXI_DAC_REG_CHAN_CNTRL_1(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_3(chan->channel);

	ret = regmap_read(st->regmap, reg, &__val);
	if (ret)
		return ret;

	frac = FIELD_GET(AXI_DAC_SCALE_FRAC, __val);
	sign = FIELD_GET(AXI_DAC_SCALE_SIGN, __val);
	integer = FIELD_GET(AXI_DAC_SCALE_INT, __val);

	vals[1] = DIV_ROUND_CLOSEST_ULL((u64)frac * MEGA, AXI_DAC_SCALE_INT);

	if (integer && sign)
		vals[0] = -1;
	else if (integer)
		vals[0] = 1;
	else if (sign)
		vals[1] *= -1;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals),
				vals);
}

static ssize_t axi_dac_phase_read(struct axi_dac_state *st,
				  const struct iio_chan_spec *chan,
				  uintptr_t private, char *buf)
{
	int vals[2] = {0};
	unsigned int tmp;
	u32 reg, __val;
	u16 phase;
	int ret;

	dev_info(st->dev, "Get phase, tone=%lu\n", private == AXI_DAC_PHASE_TONE_1 ? 1UL : 2UL);
	if (private == AXI_DAC_PHASE_TONE_1)
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan->channel);

	ret = regmap_read(st->regmap, reg, &__val);
	if (ret)
		return ret;

	phase = FIELD_GET(AXI_DAC_PHASE, __val);
	tmp = DIV_ROUND_CLOSEST_ULL((u64)phase * AXI_DAC_2_PI_MEGA, U16_MAX);
	vals[0] = tmp / MEGA;
	vals[1] = tmp % MEGA;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals),
				vals);
}

static ssize_t axi_dac_read_ext_info(struct iio_backend *back,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (private) {
	case AXI_DAC_FREQ_TONE_1:
	case AXI_DAC_FREQ_TONE_2:
		return axi_dac_frequency_read(st, private, buf);
	case AXI_DAC_SCALE_TONE_1:
	case AXI_DAC_SCALE_TONE_2:
		return axi_dac_scale_read(st, chan, private, buf);
	case AXI_DAC_PHASE_TONE_1:
	case AXI_DAC_PHASE_TONE_2:
		return axi_dac_phase_read(st, chan, private, buf);
	default:
		return -EINVAL;
	}
}

static ssize_t axi_dac_frequency_write(struct axi_dac_state *st,
				       const struct iio_chan_spec *chan,
				       uintptr_t private, const char *buf,
				       size_t len)
{
	dev_info(st->dev, "Set frequency, tone=%lu\n", private + 1);
	return len;
}

static ssize_t axi_dac_scale_write(struct axi_dac_state *st,
				   const struct iio_chan_spec *chan,
				   uintptr_t private,
				   const char *buf, size_t len)
{
	u32 scale = 0, tmp, reg;
	int integer, frac, ret;

	dev_info(st->dev, "Set scale, tone=%lu\n", private == AXI_DAC_SCALE_TONE_1 ? 1UL : 2UL);

	ret = iio_str_to_fixpoint(buf, MEGA, &integer, &frac);
	if (ret)
		return ret;

	/*  format is 1.1.14 (sign, integer and fractional bits) */
	switch (integer) {
	case 1:
		scale = FIELD_PREP(AXI_DAC_SCALE_INT, 1);
		break;
	case -1:
		scale = FIELD_PREP(AXI_DAC_SCALE_INT_NEG, 3);
		break;
	case 0:
		if (frac < 0) {
			scale = FIELD_PREP(AXI_DAC_SCALE_SIGN, 1);
			frac *= -1;
		}
		break;
	default:
		return -EINVAL;
	}

	tmp = DIV_ROUND_CLOSEST_ULL((u64)frac * AXI_DAC_SCALE_INT, MEGA);
	scale |= FIELD_PREP(AXI_DAC_SCALE_FRAC, tmp);

	if (private == AXI_DAC_SCALE_TONE_1)
		reg = AXI_DAC_REG_CHAN_CNTRL_1(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_3(chan->channel);

	guard(mutex)(&st->lock);
	ret = regmap_write(st->regmap, reg, scale);
	if (ret)
		return ret;

	/* synchronize channels */
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1, AXI_DAC_SYNC);
	if (ret)
		return ret;

	return len;
}

static ssize_t axi_dac_phase_write(struct axi_dac_state *st,
				   const struct iio_chan_spec *chan,
				   uintptr_t private,
				   const char *buf, size_t len)
{
	int integer, frac, ret;
	u32 phase, reg;

	dev_info(st->dev, "Set phase, tone=%lu\n", private == AXI_DAC_PHASE_TONE_1 ? 1UL : 2UL);

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &frac);
	if (ret)
		return ret;

	phase = integer * MEGA + frac;
	dev_info(st->dev, "Phase before is %u, integer:%d, frac:%d\n", phase,
		 integer, frac);

	phase = DIV_ROUND_CLOSEST_ULL((u64)phase * U16_MAX, AXI_DAC_2_PI_MEGA);
	dev_info(st->dev, "Phase raw %u\n", phase);

	if (private == AXI_DAC_PHASE_TONE_1)
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan->channel);

	guard(mutex)(&st->lock);
	ret = regmap_update_bits(st->regmap, reg, AXI_DAC_PHASE,
				 FIELD_PREP(AXI_DAC_PHASE, phase));
	if (ret)
		return ret;

	/* synchronize channels */
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1, AXI_DAC_SYNC);
	if (ret)
		return ret;

	return len;
}

static ssize_t axi_dac_write_ext_info(struct iio_backend *back,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (private) {
	case AXI_DAC_FREQ_TONE_1:
	case AXI_DAC_FREQ_TONE_2:
		return axi_dac_frequency_write(st, chan, private, buf, len);
	case AXI_DAC_SCALE_TONE_1:
	case AXI_DAC_SCALE_TONE_2:
		return axi_dac_scale_write(st, chan, private, buf, len);
	case AXI_DAC_PHASE_TONE_1:
	case AXI_DAC_PHASE_TONE_2:
		return axi_dac_phase_write(st, chan, private, buf, len);
	default:
		return -EINVAL;
	}

}

#define AXI_DAC_EXT_INFO(_name, _what) {	\
	.name = (_name),			\
	.private = (_what),			\
	.shared = IIO_SEPARATE,			\
}

static const struct iio_chan_spec_ext_info axi_dac_ext_info[] = {
	AXI_DAC_EXT_INFO("frequency0", AXI_DAC_FREQ_TONE_1),
	AXI_DAC_EXT_INFO("frequency1", AXI_DAC_FREQ_TONE_2),
	AXI_DAC_EXT_INFO("scale0", AXI_DAC_SCALE_TONE_1),
	AXI_DAC_EXT_INFO("scale1", AXI_DAC_SCALE_TONE_2),
	AXI_DAC_EXT_INFO("phase0", AXI_DAC_PHASE_TONE_1),
	AXI_DAC_EXT_INFO("phase1", AXI_DAC_PHASE_TONE_2),
	{}
};

static int axi_dac_get_ext_info(struct iio_backend *back,
				enum iio_chan_type chan_type,
				const struct iio_chan_spec_ext_info **ext_info,
				unsigned int *entries)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	dev_info(st->dev, "Reg config:%08X\n", st->reg_config);

	/* DDS is off so nothing to add */
	if (st->reg_config & AXI_DAC_DISABLE)
		return 0;
	if (chan_type != IIO_ALTVOLTAGE)
		return -EINVAL;

	*ext_info = axi_dac_ext_info;
	*entries = ARRAY_SIZE(axi_dac_ext_info);

	return 0;
}

static int axi_dac_data_source_set(struct iio_backend *back, unsigned int chan,
				   enum iio_backend_data_source data)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (data) {
	case IIO_BACKEND_INTERNAL_CW:
		return regmap_update_bits(st->regmap,
					  AXI_DAC_REG_CHAN_CNTRL_7(chan),
					  AXI_DAC_DATA_SEL,
					  AXI_DAC_DATA_INTERNAL_TONE);
	case IIO_BACKEND_EXTERNAL:
		return regmap_update_bits(st->regmap,
					  AXI_DAC_REG_CHAN_CNTRL_7(chan),
					  AXI_DAC_DATA_SEL, AXI_DAC_DATA_DMA);
	default:
		return -EINVAL;
	}
}

static const struct iio_backend_ops axi_dac_generic = {
	.enable = axi_dac_enable,
	.disable = axi_dac_disable,
	.request_buffer = axi_dac_request_buffer,
	.free_buffer = axi_dac_free_buffer,
	.get_ext_info = axi_dac_get_ext_info,
	.write_ext_info = axi_dac_write_ext_info,
	.read_ext_info = axi_dac_read_ext_info,
	.data_source_set = axi_dac_data_source_set,
};

static const struct regmap_config axi_dac_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int axi_dac_probe(struct platform_device *pdev)
{
	unsigned int ver, *expected_ver;
	struct axi_dac_state *st;
	void __iomem *base;
	struct clk *clk;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	expected_ver = (unsigned int *)device_get_match_data(&pdev->dev);
	if (!expected_ver)
		return -ENODEV;

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->dev = &pdev->dev;
	st->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					   &axi_dac_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	/*
	 * Force disable the core. Up to the frontend to enable us. And we can
	 * still read/write registers...
	 */
	ret = regmap_write(st->regmap, AXI_DAC_REG_RSTN, 0);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADI_AXI_REG_VERSION, &ver);
	if (ret)
		return ret;

	if (ADI_AXI_PCORE_VER_MAJOR(ver) != ADI_AXI_PCORE_VER_MAJOR(*expected_ver)) {
		dev_err(&pdev->dev,
			"Major version mismatch. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(*expected_ver),
			ADI_AXI_PCORE_VER_MINOR(*expected_ver),
			ADI_AXI_PCORE_VER_PATCH(*expected_ver),
			ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver),
			ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	/* Let's get the core read only configuration */
	ret = regmap_read(st->regmap, AXI_DAC_REG_CONFIG, &st->reg_config);
	if (ret)
		return ret;

	mutex_init(&st->lock);
	ret = devm_iio_backend_register(&pdev->dev, &axi_dac_generic, st);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "AXI DAC IP core (%d.%.2d.%c) probed\n",
		 ADI_AXI_PCORE_VER_MAJOR(ver),
		 ADI_AXI_PCORE_VER_MINOR(ver),
		 ADI_AXI_PCORE_VER_PATCH(ver));

	return 0;
}

static unsigned int axi_dac_9_1_b_info = ADI_AXI_PCORE_VER(9, 1, 'b');

static const struct of_device_id axi_dac_of_match[] = {
	{ .compatible = "adi,axi-dac-9.1.b", .data = &axi_dac_9_1_b_info },
	{}
};
MODULE_DEVICE_TABLE(of, axi_dac_of_match);

static struct platform_driver axi_dac_driver = {
	.driver = {
		.name = "adi-axi-dac",
		.of_match_table = axi_dac_of_match,
	},
	.probe = axi_dac_probe,
};
module_platform_driver(axi_dac_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic AXI DAC IP core driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
MODULE_IMPORT_NS(IIO_BACKEND);
