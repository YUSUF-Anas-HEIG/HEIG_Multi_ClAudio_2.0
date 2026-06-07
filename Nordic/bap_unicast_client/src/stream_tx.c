#include <zephyr/bluetooth/audio/bap.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>

#include "stream_lc3.h"
#include "stream_mic.h"
#include "stream_tx.h"

LOG_MODULE_REGISTER(stream_tx, LOG_LEVEL_INF);

static struct tx_stream tx_streams[CONFIG_BT_BAP_UNICAST_CLIENT_ASE_SRC_COUNT];

static bool stream_is_streaming(const struct bt_bap_stream *bap_stream)
{
    struct bt_bap_ep_info ep_info;
    if (bap_stream == NULL || bap_stream->ep == NULL) return false;
    if (bt_bap_ep_get_info(bap_stream->ep, &ep_info) != 0) return false;
    return ep_info.state == BT_BAP_EP_STATE_STREAMING;
}

static void tx_thread_func(void *arg1, void *arg2, void *arg3)
{
    NET_BUF_POOL_FIXED_DEFINE(tx_pool, CONFIG_BT_ISO_TX_BUF_COUNT,
                              BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
                              CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);
    static uint8_t mock_data[CONFIG_BT_ISO_TX_MTU];

    while (true) {
        int err = -ENOEXEC;
        for (size_t i = 0U; i < ARRAY_SIZE(tx_streams); i++) {
            struct bt_bap_stream *bap_stream = tx_streams[i].bap_stream;
            if (stream_is_streaming(bap_stream)) {
                struct net_buf *buf = net_buf_alloc(&tx_pool, K_FOREVER);
                net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

                if (IS_ENABLED(CONFIG_LIBLC3) && bap_stream->codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
                    stream_lc3_add_data(&tx_streams[i], buf);
                } else {
                    net_buf_add_mem(buf, mock_data, bap_stream->qos->sdu);
                }

                err = bt_bap_stream_send(bap_stream, buf, tx_streams[i].seq_num++);
                if (err != 0) net_buf_unref(buf);
            }
        }
        if (err != 0) k_sleep(K_MSEC(10));
    }
}

int stream_tx_register(struct bt_bap_stream *bap_stream)
{
    if (bap_stream == NULL) return -EINVAL;
    for (size_t i = 0U; i < ARRAY_SIZE(tx_streams); i++) {
        if (tx_streams[i].bap_stream == NULL) {
            tx_streams[i].bap_stream = bap_stream;
            if (IS_ENABLED(CONFIG_LIBLC3) && bap_stream->codec_cfg->id == BT_HCI_CODING_FORMAT_LC3) {
                if (stream_lc3_init(&tx_streams[i]) != 0) {
                    tx_streams[i].bap_stream = NULL;
                    return -ENOMEM;
                }
            }
            return 0;
        }
    }
    return -ENOMEM;
}

int stream_tx_unregister(struct bt_bap_stream *bap_stream)
{
    for (size_t i = 0U; i < ARRAY_SIZE(tx_streams); i++) {
        if (tx_streams[i].bap_stream == bap_stream) {
            tx_streams[i].bap_stream = NULL;
            return 0;
        }
    }
    return -ENODATA;
}

void stream_tx_init(void)
{
    static bool thread_started;
    if (!thread_started) {
        static K_KERNEL_STACK_DEFINE(tx_thread_stack, IS_ENABLED(CONFIG_LIBLC3) ? 4096U : 1024U);
        static struct k_thread tx_thread;
        k_thread_create(&tx_thread, tx_thread_stack, K_KERNEL_STACK_SIZEOF(tx_thread_stack),
                        tx_thread_func, NULL, NULL, NULL, K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
        
        if (IS_ENABLED(CONFIG_LIBLC3)) {
            stream_i2s_init();
        }
        thread_started = true;
    }
}