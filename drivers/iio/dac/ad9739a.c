// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD9739a SPI DAC driver
 *
 * Copyright 2015-2024 Analog Devices Inc.
 */
#include "linux/err.h"
#include "linux/stddef.h"
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#include <linux/iio/backend.h>
#include <linux/iio/iio.h>

#define AD9739A_REG_MODE		0
#define AD9739A_REG_MU_STAT1		0x2A
#define AD9739A_REG_ANA_CNT_1		0x32
#define AD9739A_REG_ID			0x35

#define AD9739A_REG_IS_RESERVED(reg)	\
	((reg) == 0x5 || (reg) == 0x9 || (reg) == 0x0E || (reg) == 0x0D || \
	 (reg) == 0x2B || (reg) == 0x2C || (reg) == 0x34)

struct ad9739a_state {
	struct iio_backend *back;
	struct regmap *regmap;
};

static bool ad9739a_reg_accessible(struct device *dev, unsigned int reg)
{
	if (AD9739A_REG_IS_RESERVED(reg))
		return false;
	if (reg > AD9739A_REG_MU_STAT1 && reg < AD9739A_REG_ANA_CNT_1)
		return false;

	return true;
}

static const struct regmap_config ad9739a_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.readable_reg = ad9739a_reg_accessible,
	.writeable_reg = ad9739a_reg_accessible,
	.max_register = AD9739A_REG_ID,
};

static int ad9739a_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad9739a_state *st;
	unsigned int id;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->regmap = devm_regmap_init_spi(spi, &ad9739a_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to init regmap");

	ret = regmap_read(st->regmap, AD9739A_REG_ID, &id);
	if (ret)
		return ret;

	dev_info(dev, "Device id=%04X\n", id);

	st->back = devm_iio_backend_get(dev, NULL);
	if (IS_ERR(st->back)) {
		dev_info(dev, "Defer probe...\n");
		return PTR_ERR(st->back);
	}

	return 0;
}

static const struct of_device_id ad9739a_of_match[] = {
	{ .compatible = "adi,ad9739a" },
	{}
};
MODULE_DEVICE_TABLE(of, ad9739a_of_match);

static const struct spi_device_id ad9739a_id[] = {
	{"ad9739a"},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9739a_id);

static struct spi_driver ad9739a_driver = {
	.driver = {
		.name = "ad9739a",
		.of_match_table = ad9739a_of_match,
	},
	.probe = ad9739a_probe,
	.id_table = ad9739a_id,
};
module_spi_driver(ad9739a_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9739 DAC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_BACKEND);
