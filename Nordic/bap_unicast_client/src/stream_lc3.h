
#ifndef STREAM_LC3_H
#define STREAM_LC3_H

#include <stdint.h>

#include <zephyr/bluetooth/audio/audio.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys_clock.h>

#if defined(CONFIG_LIBLC3)

#include <lc3.h>

struct stream_lc3_tx {
	uint32_t freq_hz;
	uint32_t frame_duration_us;
	uint16_t octets_per_frame;
	uint8_t frame_blocks_per_sdu;
	uint8_t chan_cnt;
	enum bt_audio_location chan_allocation;
	lc3_encoder_t encoder;
	lc3_encoder_mem_48k_t encoder_mem;
};
#endif 

struct tx_stream;


int stream_lc3_init(struct tx_stream *stream);


void stream_lc3_add_data(struct tx_stream *stream, struct net_buf *buf);

#endif 
