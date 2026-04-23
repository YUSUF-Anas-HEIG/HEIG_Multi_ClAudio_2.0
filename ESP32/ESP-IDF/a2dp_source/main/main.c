#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"

#include "driver/i2s_std.h"

static const char *TAG = "A2DP_I2S";

#define I2S_BCLK 14
#define I2S_WS   15
#define I2S_DIN  32

#define GAIN 1


static const esp_bd_addr_t target_addrs[] = {
    {0x80, 0x99, 0xE7, 0x03, 0x37, 0x33},
    {0x90, 0x7A, 0x58, 0xEC, 0x29, 0x2D},
};

#define TARGET_COUNT (sizeof(target_addrs)/sizeof(target_addrs[0]))

#define RB_SIZE (1 * 1024)

static i2s_chan_handle_t rx_chan;
static RingbufHandle_t rb;

static esp_bd_addr_t last_peer = {0};
static bool has_peer = false;

static bool mac_match(const esp_bd_addr_t a, const esp_bd_addr_t b)
{
    return memcmp(a, b, ESP_BD_ADDR_LEN) == 0;
}


static void fast_reconnect_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));

    //ESP_LOGI(TAG, "Attempting fast reconnect...");

    if (has_peer) {
        esp_a2d_source_connect(last_peer);
    } else {
        // fallback: try list
        for (int i = 0; i < TARGET_COUNT; i++) {
            if (esp_a2d_source_connect(target_addrs[i]) == ESP_OK) {
                //ESP_LOGI(TAG, "Connected to fallback device");
                break;
            }
        }
    }

    vTaskDelete(NULL);
}

static void i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT,
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_DIN,
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static void i2s_reader_task(void *arg)
{
    int32_t i2s_buf[64];
    int16_t pcm_buf[64];

    while (1) {
        size_t bytes_read = 0;

        esp_err_t err = i2s_channel_read(
            rx_chan,
            i2s_buf,
            sizeof(i2s_buf),
            &bytes_read,
            portMAX_DELAY
        );

        if (err != ESP_OK || bytes_read == 0) continue;

        int samples = bytes_read / sizeof(int32_t);

        for (int i = 0; i < samples; i++) {
            pcm_buf[i] = (int16_t)(i2s_buf[i] >> 14);
        }

        xRingbufferSend(rb, pcm_buf, samples * sizeof(int16_t), portMAX_DELAY);
    }
}


static int32_t bt_app_a2d_data_cb(uint8_t *data, int32_t len)
{
    size_t item_size;

    uint8_t *item = (uint8_t *)xRingbufferReceiveUpTo(
        rb,
        &item_size,
        0,
        len
    );

    if (!item) {
        memset(data, 0, len);
		if (len > 512) len = 512;
        return len;
    }

    int samples = (len / sizeof(int16_t)); // CHANGE int samples = item_size / sizeof(int16_t);
	if (samples > 256) samples = 256; // REMOVE IF BROKEN

    int16_t *in  = (int16_t *)item;
    int16_t *out = (int16_t *)data;

    for (int i = 0; i < samples; i++) {

        float v = in[i] * GAIN;

        if (v > 32767.0f) v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;

        out[i] = (int16_t)v;
    }

    vRingbufferReturnItem(rb, item);

    return item_size;
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event,
                          esp_a2d_cb_param_t *param)
{
    switch (event) {

    case ESP_A2D_CONNECTION_STATE_EVT:
        //ESP_LOGI(TAG, "CONNECTION STATE: %d", param->conn_stat.state);

        if (param->conn_stat.state ==
            ESP_A2D_CONNECTION_STATE_CONNECTED) {

            memcpy(last_peer,
                   param->conn_stat.remote_bda,
                   ESP_BD_ADDR_LEN);

            has_peer = true;

            //ESP_LOGI(TAG, "Connected → starting stream");

            esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
        }

        if (param->conn_stat.state ==
            ESP_A2D_CONNECTION_STATE_DISCONNECTED) {

            //ESP_LOGI(TAG, "Disconnected → fast reconnect");

            xTaskCreate(fast_reconnect_task,
                        "reconnect",
                        4096,
                        NULL,
                        5,
                        NULL);
        }

        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        //ESP_LOGI(TAG, "AUDIO STATE: %d",
        //         param->audio_stat.state);
        break;

    default:
        break;
    }
}

static void bt_init(void)
{
    esp_bt_controller_config_t bt_cfg =
        BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_a2d_register_callback(bt_app_a2d_cb));
	esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ESP_ERROR_CHECK(esp_a2d_source_register_data_callback(
        bt_app_a2d_data_cb));
    esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_SUSPEND);
    ESP_ERROR_CHECK(esp_a2d_source_init());
	//esp_a2d_media_ctrl(ESP_A2D_MEDIA_CTRL_START);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    rb = xRingbufferCreate(RB_SIZE, RINGBUF_TYPE_BYTEBUF);

    i2s_init();

    xTaskCreate(i2s_reader_task, "i2s_reader", 4096, NULL, 8, NULL);

    bt_init();

    vTaskDelay(pdMS_TO_TICKS(2000));

    //ESP_LOGI(TAG, "Connecting to target device...");
    esp_a2d_source_connect(target_addrs[0]);
}