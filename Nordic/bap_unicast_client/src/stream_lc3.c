#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
 
#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <lc3.h>
 
#include "stream_lc3.h"
#include "stream_mic.h"
#include "stream_tx.h"
 
LOG_MODULE_REGISTER(lc3, LOG_LEVEL_INF);
 
#define LC3_MAX_SAMPLE_RATE       48000U
#define LC3_MAX_FRAME_DURATION_US 10000U
#define LC3_MAX_NUM_SAMPLES \
    ((LC3_MAX_FRAME_DURATION_US * LC3_MAX_SAMPLE_RATE) / USEC_PER_SEC)


static uint32_t frame_count = 0;

#define RESET_INTERVAL_FRAMES 10000
 

static int16_t audio_buf[LC3_MAX_NUM_SAMPLES * 2U];
 

static void fill_audio_buf_i2s(struct tx_stream *stream)
{
    const unsigned int num_samples =
        (stream->lc3_tx.frame_duration_us * stream->lc3_tx.freq_hz) / USEC_PER_SEC;
    const uint8_t chan_cnt = stream->lc3_tx.chan_cnt;
 
    if (chan_cnt >= 2U) {

        int err = stream_i2s_read(audio_buf, num_samples * 2U);
 
        if (err != 0) {
            
            LOG_ERR("stream_i2s_read failed: %d — using silence", err);
            memset(audio_buf, 0, num_samples * 2U * sizeof(int16_t));
        }
    } else {
        
        int16_t stereo_buf[LC3_MAX_NUM_SAMPLES * 2U];
        int err = stream_i2s_read(stereo_buf, num_samples * 2U);
 
        if (err == 0) {
            for (unsigned int i = 0; i < num_samples; i++) {
                int32_t mixed = (int32_t)stereo_buf[i * 2U] +
                                (int32_t)stereo_buf[i * 2U + 1U];
                audio_buf[i] = (int16_t)(mixed >> 1);
            }
        } else {
            LOG_ERR("stream_i2s_read failed: %d — using silence", err);
            memset(audio_buf, 0, num_samples * sizeof(int16_t));
        }
    }
}
 
static int extract_lc3_config(struct tx_stream *stream)
{
    const struct bt_audio_codec_cfg *codec_cfg = stream->bap_stream->codec_cfg;
    struct stream_lc3_tx *lc3_tx = &stream->lc3_tx;
    int ret;
 
    LOG_INF("Extracting LC3 configuration values");
 
    ret = bt_audio_codec_cfg_get_freq(codec_cfg);
    if (ret >= 0) {
        ret = bt_audio_codec_cfg_freq_to_freq_hz(ret);
        if (ret > 0 && LC3_CHECK_SR_HZ(ret)) {
            lc3_tx->freq_hz = (uint32_t)ret;
        } else {
            LOG_ERR("Invalid frequency: %d", ret);
            return -EINVAL;
        }
    } else {
        LOG_ERR("Could not get frequency: %d", ret);
        return ret;
    }
 
    ret = bt_audio_codec_cfg_get_frame_dur(codec_cfg);
    if (ret >= 0) {
        ret = bt_audio_codec_cfg_frame_dur_to_frame_dur_us(ret);
        if (ret > 0 && LC3_CHECK_DT_US(ret)) {
            lc3_tx->frame_duration_us = (uint32_t)ret;
        } else {
            LOG_ERR("Invalid frame duration: %d", ret);
            return -EINVAL;
        }
    } else {
        LOG_ERR("Could not get frame duration: %d", ret);
        return ret;
    }
 
    ret = bt_audio_codec_cfg_get_chan_allocation(codec_cfg,
                                                  &lc3_tx->chan_allocation,
                                                  false);
    if (ret != 0) {
        lc3_tx->chan_allocation = BT_AUDIO_LOCATION_MONO_AUDIO;
    }
    lc3_tx->chan_cnt = bt_audio_get_chan_count(lc3_tx->chan_allocation);
 
    ret = bt_audio_codec_cfg_get_frame_blocks_per_sdu(codec_cfg, true);
    if (ret >= 0) {
        lc3_tx->frame_blocks_per_sdu = (uint8_t)ret;
    }
 
    ret = bt_audio_codec_cfg_get_octets_per_frame(codec_cfg);
    if (ret >= 0) {
        lc3_tx->octets_per_frame = (uint16_t)ret;
    } else {
        LOG_ERR("Could not get octets per frame: %d", ret);
        return ret;
    }
 
    return 0;
}
 
static bool encode_frame(struct tx_stream *stream, uint8_t index,
                          struct net_buf *out_buf)
{
    const uint16_t octets_per_frame = stream->lc3_tx.octets_per_frame;
    const uint8_t  chan_cnt         = stream->lc3_tx.chan_cnt;
    const int      stride           = (chan_cnt >= 2U) ? 2 : 1;
 
    int lc3_ret = lc3_encode(stream->lc3_tx.encoder,
                              LC3_PCM_FORMAT_S16,
                              audio_buf + index,
                              stride,
                              octets_per_frame,
                              net_buf_tail(out_buf));
    if (lc3_ret < 0) {
        LOG_ERR("LC3 encoder failed: %d", lc3_ret);
        return false;
    }
 
    net_buf_add(out_buf, octets_per_frame);
    return true;
}
 
static bool encode_frame_block(struct tx_stream *stream, struct net_buf *out_buf)
{
    fill_audio_buf_i2s(stream);
 
    for (uint8_t i = 0U; i < stream->lc3_tx.chan_cnt; i++) {
        if (!encode_frame(stream, i, out_buf)) {
            LOG_WRN("Failed to encode frame %u", i);
            return false;
        }
    }
 
    return true;
}
 
void stream_lc3_add_data(struct tx_stream *stream, struct net_buf *buf)
{
    for (uint8_t i = 0U; i < stream->lc3_tx.frame_blocks_per_sdu; i++) {
        if (!encode_frame_block(stream, buf)) {
            LOG_WRN("Failed to encode frame block %u", i);
            break;
        }
    }
}
 
int stream_lc3_init(struct tx_stream *stream)
{
    int err = extract_lc3_config(stream);
    if (err != 0) {
        memset(&stream->lc3_tx, 0, sizeof(stream->lc3_tx));
        return err;
    }
 
    err = stream_i2s_start(stream->lc3_tx.freq_hz);
    if (err != 0) {
        LOG_ERR("stream_i2s_start failed: %d", err);
        memset(&stream->lc3_tx, 0, sizeof(stream->lc3_tx));
        return err;
    }
 
    LOG_INF("Setting up LC3 encoder");
    stream->lc3_tx.encoder =
        lc3_setup_encoder(stream->lc3_tx.frame_duration_us,
                          stream->lc3_tx.freq_hz,
                          0,
                          &stream->lc3_tx.encoder_mem);
 
    if (stream->lc3_tx.encoder == NULL) {
        LOG_ERR("Failed to setup LC3 encoder");
        return -ENOEXEC;
    }
 
    return 0;
}
 