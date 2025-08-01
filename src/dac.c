// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/dac.h>

#if !DT_NODE_HAS_PROP(DT_PATH(zephyr_user), dac) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), dac_channel_id) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), dac_resolution)

#warning "DAC properties missing in devicetree"
int dac_init(void){return -1;}
int dac_write(uint32_t value){return 0;}
#else

#define DAC_NODE DT_PHANDLE(DT_PATH(zephyr_user), dac)
#define DAC_CHANNEL_ID DT_PROP(DT_PATH(zephyr_user), dac_channel_id)
#define DAC_RESOLUTION DT_PROP(DT_PATH(zephyr_user), dac_resolution)

static const struct device *const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg =
{
    .channel_id = DAC_CHANNEL_ID,
    .resolution = DAC_RESOLUTION,
    .buffered = true
};

int dac_init(void)
{
    if(!device_is_ready(dac_dev))
    {
        return -ENODEV;
    }
    return dac_channel_setup(dac_dev, &dac_ch_cfg);
}

int dac_write(uint32_t value)
{
    return dac_write_value(dac_dev, DAC_CHANNEL_ID, value);
}
#endif