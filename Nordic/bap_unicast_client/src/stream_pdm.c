// Reworked from I2S code iteration
#include "stream_mic.h"
 
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
 
#include <zephyr/device.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/__assert.h>
 
LOG_MODULE_REGISTER(stream_pdm, LOG_LEVEL_INF);
 

#define PDM_NUM_BLOCKS          3U
 
#define PDM_MAX_BLOCK_BYTES     8192U
 
#define RING_BUF_BYTES          2048U

#define WATERMARK_BLOCKS        4U

#define BUFFER_SIZE_BYTES 2048 
#define WATERMARK_HIGH    (RING_BUF_BYTES * 9 / 10) 
#define WATERMARK_LOW     (RING_BUF_BYTES * 1 / 10) 
 
#define CAPTURE_THREAD_STACK_SIZE   2048U
#define CAPTURE_THREAD_PRIORITY     K_PRIO_PREEMPT(2)
 

static struct k_mem_slab pdm_mem_slab;
static uint8_t __aligned(4) pdm_slab_buf[PDM_NUM_BLOCKS * PDM_MAX_BLOCK_BYTES];
 
static uint8_t  ring_buf_data[RING_BUF_BYTES];
static struct   ring_buf rb;
 

static volatile uint32_t configured_block_bytes;
 
static K_SEM_DEFINE(sem_data_ready, 0, K_SEM_MAX_LIMIT);
static K_SEM_DEFINE(sem_bt_ready,   0, 1);
 
static bool           thread_created;
static K_MUTEX_DEFINE(init_mutex);
static const struct   device *pdm_dev_ptr;
 
static K_THREAD_STACK_DEFINE(capture_thread_stack, CAPTURE_THREAD_STACK_SIZE);
static struct k_thread capture_thread_data;
 

static void push_block_to_ring(const int16_t *samples, size_t sample_count)
{
    const uint32_t bytes_to_write  = sample_count * sizeof(int16_t);
    const uint32_t watermark_bytes = WATERMARK_BLOCKS * configured_block_bytes;
 
    if (ring_buf_size_get(&rb) + bytes_to_write > watermark_bytes &&
        configured_block_bytes > 0U) {
 
        uint8_t *claim_buf;
        uint32_t claimed = ring_buf_get_claim(&rb, &claim_buf,
                                              configured_block_bytes);
        if (claimed > 0U) {
            ring_buf_get_finish(&rb, claimed);
            LOG_DBG("Clock drift: evicted %u B from ring buffer", claimed);
        }
    }
 
    const uint32_t written = ring_buf_put(&rb, (const uint8_t *)samples,
                                          bytes_to_write);
 
    if (written < bytes_to_write) {
        LOG_WRN("Ring buffer full after eviction: dropped %u B",
                bytes_to_write - written);
    }

    if (ring_buf_size_get(&rb) >= 1280) {
        k_sem_give(&sem_data_ready);
    }
}

static int16_t apply_gain(int16_t sample, float gain) {

    float boosted = (float)sample * gain;
    
    if (boosted > 32767.0f) return 32767;
    if (boosted < -32768.0f) return -32768;
    
    return (int16_t)boosted;
}
 
static void capture_thread_func(void *arg1, void *arg2, void *arg3)
{
    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
 
    int err;
 
    LOG_INF("PDM capture thread waiting for BT");
    k_sem_take(&sem_bt_ready, K_FOREVER);
    LOG_INF("PDM capture thread starting DMA");
 
    const struct device *dev = pdm_dev_ptr;
    
    err = dmic_trigger(dev, DMIC_TRIGGER_START);
    if (err != 0) {
        LOG_ERR("DMIC_TRIGGER_START failed: %d", err);
        return;
    }
    static uint8_t dummy[2];
    while (true) {
        void  *rx_buf  = NULL;
        size_t rx_size = 0;

        static uint32_t last_log_time = 0;
        uint32_t now = k_uptime_get_32();

        // Log every 5 seconds, not every loop iteration (to avoid flooding RTT)
        if (now - last_log_time > 5000) {
            LOG_INF("Buffer Occupancy: %u bytes", ring_buf_size_get(&rb));
            last_log_time = now;
        }
 
        err = dmic_read(dev, 0, &rx_buf, &rx_size, 500);
 
        if (err == -EAGAIN || err == -ETIMEDOUT) {
            LOG_ERR("PDM stall detected — resetting");
            dmic_trigger(dev, DMIC_TRIGGER_STOP);
            k_sleep(K_MSEC(10));
            dmic_trigger(dev, DMIC_TRIGGER_START);
            continue;
        }
 
        if (err != 0) {
            LOG_ERR("dmic_read error: %d", err);
            continue;
        }
 
        if (rx_buf == NULL || rx_size == 0) {
            LOG_WRN("Empty PDM block");
            continue;
        }

        int16_t *samples = (int16_t *)rx_buf;
        size_t num_samples = rx_size / sizeof(int16_t);

        for (size_t i = 0; i < num_samples; i++) {
            samples[i] = apply_gain(samples[i], 4.f); // 1.5x gain
        }
 
        push_block_to_ring((const int16_t *)rx_buf, rx_size / sizeof(int16_t));
        k_mem_slab_free(&pdm_mem_slab, rx_buf);

        // TO REMOVE
        uint32_t occupancy = ring_buf_size_get(&rb);
        static uint32_t drop_count = 0;
        if (occupancy > 2000) {
            uint8_t dummy[2];
            ring_buf_get(&rb, dummy, 2);
        }
        if (occupancy > WATERMARK_HIGH) {
            drop_count++;
            LOG_WRN("Governor Drop #%u", drop_count);
            reset_pdm_stream();
        }
        else if (occupancy < WATERMARK_LOW) {
            k_sleep(K_MSEC(5)); // Give BT stack room to breathe
        }
    }
}
 

int stream_i2s_init(void)
{
    k_mutex_lock(&init_mutex, K_FOREVER);
 
    if (thread_created) {
        k_mutex_unlock(&init_mutex);
        return 0;
    }
 
    pdm_dev_ptr = DEVICE_DT_GET(DT_ALIAS(mic_pdm));
    if (!device_is_ready(pdm_dev_ptr)) {
        LOG_ERR("PDM device not ready");
        k_mutex_unlock(&init_mutex);
        return -ENODEV;
    }
 
    memset(&pdm_mem_slab, 0, sizeof(pdm_mem_slab));
    ring_buf_init(&rb, RING_BUF_BYTES, ring_buf_data);
 
    k_thread_create(&capture_thread_data,
                    capture_thread_stack,
                    K_THREAD_STACK_SIZEOF(capture_thread_stack),
                    capture_thread_func,
                    NULL, NULL, NULL,
                    CAPTURE_THREAD_PRIORITY,
                    0, K_NO_WAIT);
    k_thread_name_set(&capture_thread_data, "pdm_capture");
 
    thread_created = true;
    k_mutex_unlock(&init_mutex);
 
    LOG_INF("PDM phase-1 init done");
    return 0;
}
 
int stream_i2s_start(uint32_t sample_rate_hz)
{
    static bool is_configured;
 
    if (is_configured) {
        return 0;
    }
 
    uint32_t samples_per_channel = sample_rate_hz / 100U;
    uint32_t block_bytes = samples_per_channel * 2U * sizeof(int16_t);
 
    if (block_bytes > PDM_MAX_BLOCK_BYTES) {
        LOG_ERR("Block size %u B exceeds maximum %u B",
                block_bytes, PDM_MAX_BLOCK_BYTES);
        return -EINVAL;
    }
 
    int err = k_mem_slab_init(&pdm_mem_slab, pdm_slab_buf,
                              block_bytes, PDM_NUM_BLOCKS);
    if (err != 0) {
        LOG_ERR("k_mem_slab_init failed: %d", err);
        return err;
    }
 
    configured_block_bytes = block_bytes;
 
    struct pcm_stream_cfg mic_stream_cfg = {
        .pcm_rate   = sample_rate_hz,
        .pcm_width  = 16U,
        .block_size = block_bytes,
        .mem_slab   = &pdm_mem_slab,
    };
 
    struct dmic_cfg cfg = {
        .io = {
            .min_pdm_clk_freq = 1000000U,   
            .max_pdm_clk_freq = 3500000U,
            .min_pdm_clk_dc   = 40U,
            .max_pdm_clk_dc   = 60U,
        },
        .streams = &mic_stream_cfg,
        .channel = {
            .req_num_streams = 1U,
            .req_num_chan    = 2U,
            .req_chan_map_lo =
                dmic_build_channel_map(0, 0, PDM_CHAN_LEFT) |
                dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT),
        },
    };
 
    err = dmic_configure(pdm_dev_ptr, &cfg);
    if (err != 0) {
        LOG_ERR("dmic_configure failed: %d", err);
        return err;
    }
 
    LOG_INF("PDM configured: %u B / 10 ms at %u Hz", block_bytes, sample_rate_hz);
 
    is_configured = true;
    k_sem_give(&sem_bt_ready);
    return 0;
}
 
int stream_i2s_read(int16_t *buf, size_t num_samples)
{
    uint32_t bytes_needed = num_samples * sizeof(int16_t);
    uint32_t bytes_read   = 0U;
 
    while (bytes_read < bytes_needed) {
        uint32_t got = ring_buf_get(&rb,
                                    (uint8_t *)buf + bytes_read,
                                    bytes_needed - bytes_read);
        bytes_read += got;
 
        if (bytes_read < bytes_needed) {
            k_sem_take(&sem_data_ready, K_FOREVER);
        }
    }
 
    return 0;
}
 
uint32_t stream_i2s_buf_occupancy(void)
{
    return ring_buf_size_get(&rb);
}

void reset_pdm_stream(void)
{

    dmic_trigger(pdm_dev_ptr, DMIC_TRIGGER_STOP);
    
    ring_buf_reset(&rb);

    dmic_trigger(pdm_dev_ptr, DMIC_TRIGGER_START);
}

void partial_reset_pdm_stream(void)
{

    ring_buf_reset(&rb);

    uint8_t silence[1920] = {0};
    ring_buf_put(&rb, silence, 1920);
}

// Cleaner version of reset_pdm_stream()
void stream_i2s_reset(void)
{
    
    if (pdm_dev_ptr != NULL && device_is_ready(pdm_dev_ptr)) {
        dmic_trigger(pdm_dev_ptr, DMIC_TRIGGER_STOP);
    }
    
    ring_buf_reset(&rb);
    
    
    k_sem_reset(&sem_data_ready);
    k_sem_reset(&sem_bt_ready);
    
    
    if (configured_block_bytes > 0U) {
        k_mem_slab_init(&pdm_mem_slab, pdm_slab_buf, 
                        configured_block_bytes, PDM_NUM_BLOCKS);
    }

}