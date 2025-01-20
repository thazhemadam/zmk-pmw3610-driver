/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */
#define DT_DRV_COMPAT pixart_pmw3360

#include <drivers/spi.h>
#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <sys/byteorder.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#ifdef CONFIG_PMW3360_3389
#include <pmw3389_srom.h>
#elif CONFIG_PMW3360_3360
#include <pmw3360_srom.h>
#endif

#include "pmw3360.h"

LOG_MODULE_REGISTER(PMW3360, CONFIG_SENSOR_LOG_LEVEL);
#define PMW3360_PID COND_CODE_1(CONFIG_PMW3360_3389, (PMW3360_3389_PID), (PMW3360_3360_PID))
#define PMW3360_CPI_MAX                                                                            \
    COND_CODE_1(CONFIG_PMW3360_3389, (PMW3360_3389_CPI_MAX), (PMW3360_3360_CPI_MAX))
#define PMW3360_CPI_MIN                                                                            \
    COND_CODE_1(CONFIG_PMW3360_3389, (PMW3360_3389_CPI_MIN), (PMW3360_3360_CPI_MIN))

struct pmw3360_motion_burst {
    uint8_t motion;
    uint8_t observation;
    int16_t dx;
    int16_t dy;
} __attribute__((packed));

static inline int pmw3360_cs_select(const struct pmw3360_gpio_dt_spec *cs_gpio_cfg,
                                    const uint8_t value) {
    return gpio_pin_set(cs_gpio_cfg->port, cs_gpio_cfg->pin, value);
}

static int pmw3360_access(const struct device *dev, const uint8_t reg, uint8_t *value) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *cfg = dev->config;
    const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
    const struct pmw3360_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    uint8_t access[1] = {reg};
    struct spi_buf_set tx = {
        .buffers =
            &(struct spi_buf){
                .buf = access,
                .len = 1,
            },
        .count = 1,
    };

    uint8_t result[1];
    if (value != NULL) {
        result[0] = *value;
    }
    struct spi_buf_set rx = {
        .buffers =
            &(struct spi_buf){
                .buf = result,
                .len = 1,
            },
        .count = 1,
    };

    pmw3360_cs_select(cs_gpio_cfg, 0);

    int err = spi_write(data->bus, spi_cfg, &tx);
    k_sleep(K_USEC(120)); // Tsrad
    if (err) {
        pmw3360_cs_select(cs_gpio_cfg, 1);
        return err;
    }

    if ((reg & PMW3360_WR_MASK))
        err = spi_write(data->bus, spi_cfg, &rx);
    else
        err = spi_read(data->bus, spi_cfg, &rx);
    pmw3360_cs_select(cs_gpio_cfg, 1);
    k_sleep(K_USEC(160));
    if ((reg & PMW3360_WR_MASK) == 0 && value != NULL)
        *value = result[0];
    return err;
}
static int pmw3360_read_reg(const struct device *dev, const uint8_t reg, uint8_t *value) {
    return pmw3360_access(dev, reg & PMW3360_RD_MASK, value);
}
static int pmw3360_write_reg(const struct device *dev, const uint8_t reg, const uint8_t value) {
    uint8_t v = value;
    return pmw3360_access(dev, reg | PMW3360_WR_MASK, &v);
}

static int pmw3360_write_srom(const struct device *dev) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *cfg = dev->config;
    const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
    const struct pmw3360_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;
    uint8_t access[1] = {PMW3360_REG_SROM_BURST | PMW3360_WR_MASK};
    struct spi_buf_set tx = {
        .buffers =
            &(struct spi_buf){
                .buf = access,
                .len = 1,
            },
        .count = 1,
    };

    pmw3360_write_reg(dev, PMW3360_REG_SROM_EN, PMW3360_SROM_DWNLD_CMD);
    k_sleep(K_USEC(15));
    pmw3360_write_reg(dev, PMW3360_REG_SROM_EN, PMW3360_SROM_DWNLD_START_CMD);

    pmw3360_cs_select(cs_gpio_cfg, 0);

    int err = spi_write(data->bus, spi_cfg, &tx);

    k_sleep(K_USEC(15));
    if (err) {
        pmw3360_cs_select(cs_gpio_cfg, 1);
        return err;
    }

    for (uint16_t i = 0; i < sizeof(SROM); i++) {
        access[0] = SROM[i];
        err = spi_write(data->bus, spi_cfg, &tx);
        k_sleep(K_USEC(15));
        if (err) {
            pmw3360_cs_select(cs_gpio_cfg, 1);
            return err;
        }
    }

    pmw3360_cs_select(cs_gpio_cfg, 1);
    k_sleep(K_MSEC(2)); // Tbexit
    return err;
}

static int pmw3360_read_motion_burst(const struct device *dev, struct pmw3360_motion_burst *burst) {
    struct pmw3360_data *data = dev->data;
    const struct pmw3360_config *cfg = dev->config;
    const struct spi_config *spi_cfg = &cfg->bus_cfg.spi_cfg->spi_conf;
    const struct pmw3360_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    uint8_t access[1] = {PMW3360_REG_BURST};
    struct spi_buf_set tx = {
        .buffers =
            &(struct spi_buf){
                .buf = access,
                .len = 1,
            },
        .count = 1,
    };
    struct spi_buf_set rx = {
        .buffers =
            &(struct spi_buf){
                .buf = (uint8_t *)burst,
                .len = sizeof(struct pmw3360_motion_burst),
            },
        .count = 1,
    };

    pmw3360_cs_select(cs_gpio_cfg, 0);

    int err = spi_write(data->bus, spi_cfg, &tx);
    k_sleep(K_USEC(35)); // tsrad motbr
    if (err) {
        pmw3360_cs_select(cs_gpio_cfg, 1);
        return err;
    }
    err = spi_read(data->bus, spi_cfg, &rx);
    pmw3360_cs_select(cs_gpio_cfg, 1);
#ifdef CONFIG_PMW3360_TRIGGER
    pmw3360_reset_motion(dev);
#endif
    return err;
}

#ifdef CONFIG_PMW3360_TRIGGER
void pmw3360_reset_motion(const struct device *dev) {
    // reset motswk interrupt
    pmw3360_read_reg(dev, PMW3360_REG_MOTION, NULL);
}
#endif

int pmw3360_spi_init(const struct device *dev) {
    const struct pmw3360_config *cfg = dev->config;
    const struct pmw3360_gpio_dt_spec *cs_gpio_cfg = &cfg->bus_cfg.spi_cfg->cs_spec;

    int err;
    err = gpio_pin_configure(cs_gpio_cfg->port, cs_gpio_cfg->pin, GPIO_OUTPUT_ACTIVE);
    if (err) {
        LOG_ERR("could configure cs pin %d", err);
        return -EIO;
    }
    return 0;
}

static int pmw3360_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct pmw3360_data *data = dev->data;
    struct pmw3360_motion_burst burst;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_DX && chan != SENSOR_CHAN_POS_DY)
        return -ENOTSUP;

    int err = pmw3360_read_motion_burst(dev, &burst);
    if (err) {
        return err;
    }
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POS_DX)
        data->dx += burst.dx;
    if (chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_POS_DY)
        data->dy += burst.dy;
    return 0;
}

static int pmw3360_channel_get(const struct device *dev, enum sensor_channel chan,
                               struct sensor_value *val) {
    struct pmw3360_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_POS_DX:
        val->val1 = data->dx;
        data->dx = 0;
        break;
    case SENSOR_CHAN_POS_DY:
        val->val1 = data->dy;
        data->dy = 0;
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api pmw3360_driver_api = {
#ifdef CONFIG_PMW3360_TRIGGER
    .trigger_set = pmw3360_trigger_set,
#endif
    // eventually implement this to allow setting cpi from the driver api maybe
    // .attr_set = pmw3360_attr_set,
    .sample_fetch = pmw3360_sample_fetch,
    .channel_get = pmw3360_channel_get,
};

static int pmw3360_set_cpi(const struct device *dev, uint16_t cpi) {
#ifdef CONFIG_PMW3360_3360
    if (cpi > PMW3360_CPI_MIN && cpi < PMW3360_CPI_MAX)
        return pmw3360_write_reg(dev, PMW3360_3360_REG_CPI,
                                 ((cpi / 100) - 1)); /* 100-12000, 100 cpi LSb */
#elif CONFIG_PMW3360_3389
    if (cpi > PMW3360_CPI_MIN && cpi < PMW3360_CPI_MAX) {
        cpi /= 50;
        int err =
            pmw3360_write_reg(dev, PMW3360_3389_REG_CPI_L, cpi & 0xFF); /* 50-16000, 50 cpi LSb */
        if (err) {
            return err;
        }
        return pmw3360_write_reg(dev, PMW3360_3389_REG_CPI_H, cpi >> 8); /* 50-16000, 50 cpi LSb */
    }
#endif
    return -ENOTSUP;
}
static int pmw3360_init_chip(const struct device *dev) {
    const struct pmw3360_config *const config = dev->config;
    const struct pmw3360_gpio_dt_spec *cs_gpio_cfg = &config->bus_cfg.spi_cfg->cs_spec;
    pmw3360_cs_select(cs_gpio_cfg, 1);
    k_sleep(K_MSEC(1));

    int err = pmw3360_write_reg(dev, PMW3360_REG_PWR_UP_RST, PMW3360_RESET_CMD);
    if (err) {
        LOG_ERR("could not reset %d", err);
        return -EIO;
    }
    uint8_t pid = 0x0;
    err = pmw3360_read_reg(dev, PMW3360_REG_PID, &pid);
    if (err) {
        LOG_ERR("could not reset %d", err);
        return -EIO;
    }
    if (pid != PMW3360_PID) {
        LOG_ERR("pid does not match expected: got (%x), expected(%x)", pid, PMW3360_PID);
        return -EIO;
    }
    pmw3360_write_reg(dev, PMW3360_REG_CONFIG2,
                      config->disable_rest ? 0x00 : PMW3360_RESTEN); // set rest enable

    err = pmw3360_write_srom(dev);
    if (err) {
        LOG_ERR("could not upload srom %d", err);
        return -EIO;
    }
    uint8_t srom_run = 0x0;
    err = pmw3360_read_reg(dev, PMW3360_REG_OBSERVATION, &srom_run);
    if (err) {
        LOG_ERR("could not check srom status %d", err);
        return -EIO;
    }
    if (!(srom_run & PMW3360_SROM_RUN)) {
        LOG_ERR("srom status invalid %d", srom_run);
        return -EIO;
    }

    uint8_t srom_id = 0x0;
    err = pmw3360_read_reg(dev, PMW3360_REG_SROM_ID, &srom_id);
    if (err) {
        LOG_ERR("could not check srom id %d", err);
        return -EIO;
    }
    if (!srom_id) {
        LOG_ERR("srom id invalid %d", srom_id);
        return -EIO;
    }

    pmw3360_write_reg(dev, PMW3360_REG_BURST, 0x01);
    struct pmw3360_motion_burst val;
    pmw3360_read_motion_burst(dev, &val); // read and throwout initial motion data

    if (config->cpi > PMW3360_CPI_MIN && config->cpi < PMW3360_CPI_MAX)
        return pmw3360_set_cpi(dev, config->cpi);
    return 0;
}

static int pmw3360_init(const struct device *dev) {
    const struct pmw3360_config *const config = dev->config;
    struct pmw3360_data *data = dev->data;

    data->bus = device_get_binding(config->bus_name);
    if (!data->bus) {
        LOG_DBG("master not found: %s", log_strdup(config->bus_name));
        return -EINVAL;
    }

    config->bus_init(dev);

    if (pmw3360_init_chip(dev) < 0) {
        LOG_DBG("failed to initialize chip");
        return -EIO;
    }

#ifdef CONFIG_PMW3360_TRIGGER
    if (pmw3360_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }
#endif

    return 0;
}

#define PMW3360_DATA_SPI(n)                                                                        \
    { .cs_ctrl = {}, }

#define PMW3360_SPI_CFG(n)                                                                         \
    (&(struct pmw3360_spi_cfg){                                                                    \
        .spi_conf =                                                                                \
            {                                                                                      \
                .frequency = DT_INST_PROP(n, spi_max_frequency),                                   \
                .operation =                                                                       \
                    (SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA),        \
                .slave = DT_INST_REG_ADDR(n),                                                      \
            },                                                                                     \
        .cs_spec = PMW3360_GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), cs_gpios, 0),                   \
    })

#define PMW3360_GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                                        \
    {                                                                                              \
        .port = DEVICE_DT_GET(DT_GPIO_CTLR_BY_IDX(node_id, prop, idx)),                            \
        .pin = DT_GPIO_PIN_BY_IDX(node_id, prop, idx),                                             \
        .dt_flags = DT_GPIO_FLAGS_BY_IDX(node_id, prop, idx),                                      \
    }

#define PMW3360_CONFIG_SPI(n)                                                                      \
    {                                                                                              \
        .bus_name = DT_INST_BUS_LABEL(n), .bus_init = pmw3360_spi_init,                            \
        .bus_cfg = {.spi_cfg = PMW3360_SPI_CFG(n)},                                                \
        .disable_rest = DT_INST_NODE_HAS_PROP(n, disable_rest),                                    \
        COND_CODE_0(DT_INST_NODE_HAS_PROP(n, cpi), (0), (DT_INST_PROP(n, cpi)))                    \
            COND_CODE_1(CONFIG_PMW3360_TRIGGER,                                                    \
                        (, PMW3360_GPIO_DT_SPEC_GET_BY_IDX(DT_DRV_INST(n), motswk_gpios, 0)), ())  \
    }

#define PMW3360_INST(n)                                                                            \
    static struct pmw3360_data pmw3360_data_##n = PMW3360_DATA_SPI(n);                             \
    static const struct pmw3360_config pmw3360_cfg_##n = PMW3360_CONFIG_SPI(n);                    \
    DEVICE_DT_INST_DEFINE(n, pmw3360_init, device_pm_control_nop, &pmw3360_data_##n,               \
                          &pmw3360_cfg_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,              \
                          &pmw3360_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_INST)
