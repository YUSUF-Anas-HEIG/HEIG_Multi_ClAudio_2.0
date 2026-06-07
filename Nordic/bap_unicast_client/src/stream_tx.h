
#ifndef STREAM_TX_H
#define STREAM_TX_H

#include <stdint.h>

#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/audio/cap.h>
#include <zephyr/types.h>

#include "stream_lc3.h"

struct tx_stream {
	struct bt_bap_stream *bap_stream;
	uint16_t seq_num;

#if defined(CONFIG_LIBLC3)
	struct stream_lc3_tx lc3_tx;
#endif
};

void stream_tx_init(void);

int stream_tx_register(struct bt_bap_stream *bap_stream);

int stream_tx_unregister(struct bt_bap_stream *bap_stream);

#endif 
