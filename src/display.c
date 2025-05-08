// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/auxdisplay.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(auxdisplay, LOG_LEVEL_DBG);

static const struct device *const display_dev = DEVICE_DT_GET(DT_NODELABEL(auxdisplay_0));

int display_init()
{
    if(!device_is_ready(display_dev))
    {
        LOG_ERR("Auxdisplay device is not ready.");
        return -ENODEV;
    }
    return 0;
}

int display_write(const uint8_t *data, uint16_t len)
{
    return auxdisplay_write(display_dev, data, len);
}

int display_clear(void)
{
    return auxdisplay_clear(display_dev);
}

int display_set_cursor(bool state)
{
    return auxdisplay_cursor_set_enabled(display_dev, state);
}

int display_set_backlight(uint8_t state)
{
    return auxdisplay_backlight_set(display_dev, state);
}
