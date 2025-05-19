// Copyright (c) 2025
// SPDX-License-Identifier: Apache-2.0
// Coskun ERGAN <coskunergan@gmail.com>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

const struct adc_dt_spec adc_channels[] =
{
    DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

const size_t adc_channels_len = ARRAY_SIZE(adc_channels);

const struct adc_dt_spec *get_adc_channels(void)
{
    return adc_channels;
}

const size_t get_adc_channels_len(void)
{
    return adc_channels_len;
}

typedef void (*adc_callback_t)(void *, size_t, int16_t);

struct adc_isr_context
{
    struct k_work work;
    int16_t buffer;
    int16_t *sample;
    adc_callback_t done_cb;
    adc_callback_t done_cb_isr;
    enum adc_action state;
    struct adc_obj *self;
    void *user_data;
};

struct adc_obj
{
    struct adc_sequence_options options;
    struct adc_sequence sequence;
    size_t channel_count;
    size_t channel_index;
    struct adc_isr_context isr_context;
};

static void adc_soft_isr(struct k_work *work);
static enum adc_action adc_hard_isr(const struct device *dev, const struct adc_sequence *seq, uint16_t sampling_index);

struct adc_obj *adc_new(void)
{
    struct adc_obj *adc = k_malloc(sizeof(struct adc_obj));
    if(!adc)
    {
        return NULL;
    }

    adc->channel_count = adc_channels_len;
    if(adc->channel_count == 0)
    {
        k_free(adc);
        return NULL;
    }

    for(size_t i = 0; i < adc->channel_count; i++)
    {
        if(!device_is_ready(adc_channels[i].dev))
        {
            k_free(adc);
            return NULL;
        }
        int err = adc_channel_setup_dt(&adc_channels[i]);
        if(err < 0)
        {
            k_free(adc);
            return NULL;
        }
    }

    adc->isr_context.sample = k_malloc(adc->channel_count * sizeof(int16_t));
    if(!adc->isr_context.sample)
    {
        k_free(adc);
        return NULL;
    }
    for(size_t i = 0; i < adc->channel_count; i++)
    {
        adc->isr_context.sample[i] = 0;
    }

    adc->channel_index = 0;
    adc->isr_context.buffer = 0;
    adc->isr_context.done_cb = NULL;
    adc->isr_context.done_cb_isr = NULL;
    adc->isr_context.state = ADC_ACTION_CONTINUE;
    adc->isr_context.self = adc;
    adc->isr_context.user_data = NULL;

    k_work_init(&adc->isr_context.work, adc_soft_isr);

    adc->options = (struct adc_sequence_options)
    {
        .interval_us = 0,
        .callback = adc_hard_isr,
        .user_data = &adc->isr_context,
        .extra_samplings = 0,
    };

    return adc;
}

int adc_async_read(struct adc_obj *adc, uint32_t interval_us, adc_callback_t handler, void *user_data)
{
    if(!adc || adc->channel_index >= adc->channel_count)
    {
        return -EINVAL;
    }

    adc->options.interval_us = interval_us;

    adc->sequence = (struct adc_sequence)
    {
        .options = &adc->options,
        .channels = BIT(adc_channels[adc->channel_index].channel_cfg.channel_id),
        .buffer = &adc->isr_context.buffer,
        .buffer_size = sizeof(adc->isr_context.buffer),
        .resolution = adc_channels[adc->channel_index].resolution,
        .calibrate = false,
        .oversampling = 0,
    };

    adc->isr_context.done_cb = handler;
    adc->isr_context.done_cb_isr = NULL;
    adc->isr_context.state = ADC_ACTION_CONTINUE;
    adc->isr_context.user_data = user_data;

    return adc_read_async(adc_channels[adc->channel_index].dev, &adc->sequence, NULL);
}

int adc_async_read_isr(struct adc_obj *adc, uint32_t interval_us, adc_callback_t handler, void *user_data)
{
    if(!adc || adc->channel_index >= adc->channel_count)
    {
        return -EINVAL;
    }

    adc->options.interval_us = interval_us;

    adc->sequence = (struct adc_sequence)
    {
        .options = &adc->options,
        .channels = BIT(adc_channels[adc->channel_index].channel_cfg.channel_id),
        .buffer = &adc->isr_context.buffer,
        .buffer_size = sizeof(adc->isr_context.buffer),
        .resolution = adc_channels[adc->channel_index].resolution,
        .calibrate = false,
        .oversampling = 0,
    };

    adc->isr_context.done_cb = NULL;
    adc->isr_context.done_cb_isr = handler;
    adc->isr_context.state = ADC_ACTION_CONTINUE;
    adc->isr_context.user_data = user_data;

    return adc_read_async(adc_channels[adc->channel_index].dev, &adc->sequence, NULL);
}

void adc_cancel_read(struct adc_obj *adc)
{
    if(adc)
    {
        adc->isr_context.state = ADC_ACTION_FINISH;
    }
}

int32_t adc_get_voltage(struct adc_obj *adc, size_t idx)
{
    if(!adc || idx >= adc->channel_count)
    {
        return 0;
    }
    int32_t val_mv = adc->isr_context.sample[idx];
    if(adc_channels[idx].channel_cfg.differential)
    {
        val_mv = (int32_t)((int16_t)val_mv);
    }
    adc_raw_to_millivolts_dt(&adc_channels[idx], &val_mv);
    return val_mv;
}

int32_t adc_get_value(struct adc_obj *adc, size_t idx)
{
    if(!adc || idx >= adc->channel_count)
    {
        return 0;
    }
    int32_t val = adc->isr_context.sample[idx];
    if(adc_channels[idx].channel_cfg.differential)
    {
        val = (int32_t)((int16_t)val);
    }
    return val;
}

void adc_free(struct adc_obj *adc)
{
    if(adc)
    {
        k_free(adc->isr_context.sample);
        k_free(adc);
    }
}

static void adc_soft_isr(struct k_work *work)
{
    struct adc_isr_context *context = CONTAINER_OF(work, struct adc_isr_context, work);
    struct adc_obj *adc = context->self;

    if(adc->channel_count > 1)
    {
        size_t next_index = adc->channel_index;
        if(next_index < adc->channel_count)
        {
            adc->sequence.channels = BIT(adc_channels[next_index].channel_cfg.channel_id);
            adc->sequence.resolution = adc_channels[next_index].resolution;
            int res = adc_read_async(adc_channels[next_index].dev, &adc->sequence, NULL);
            if(res != 0)
            {
                return;
            }
        }
    }

    if(context->done_cb)
    {
        size_t idx = (adc->channel_index == 0) ? (adc->channel_count - 1) : (adc->channel_index - 1);
        if(idx < adc->channel_count)
        {
            context->done_cb(context->user_data, idx, context->sample[idx]);
        }
    }
}

static enum adc_action adc_hard_isr(const struct device *dev, const struct adc_sequence *seq, uint16_t sampling_index)
{
    struct adc_isr_context *context = seq->options->user_data;
    struct adc_obj *adc = context->self;

    if(adc->channel_index < adc->channel_count)
    {
        context->sample[adc->channel_index] = context->buffer;
    }

    if(context->state != ADC_ACTION_FINISH)
    {
        if(adc->channel_count == 1)
        {
            context->state = ADC_ACTION_REPEAT;
            if(context->done_cb_isr)
            {
                context->done_cb_isr(context->user_data, 0, context->sample[0]);
            }
            else
            {
                k_work_submit(&context->work);
            }
        }
        else if(++adc->channel_index < adc->channel_count)
        {
            context->state = ADC_ACTION_CONTINUE;
            if(context->done_cb_isr)
            {
                context->done_cb_isr(context->user_data, adc->channel_index - 1, context->sample[adc->channel_index - 1]);
            }
            else
            {
                k_work_submit(&context->work);
            }
        }
        else
        {
            if(context->state != ADC_ACTION_REPEAT)
            {
                context->state = ADC_ACTION_REPEAT;
                adc->channel_index--;
            }
            else
            {
                context->state = ADC_ACTION_CONTINUE;
                adc->channel_index = 0;
                if(context->done_cb_isr)
                {
                    context->done_cb_isr(context->user_data, adc->channel_count - 1, context->sample[adc->channel_count - 1]);
                }
                else
                {
                    k_work_submit(&context->work);
                }
            }
        }
    }

    return context->state;
}