idf_component_register(SRCS "bt_app_core.c"
                            "main.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_i2s esp_ringbuf
                    INCLUDE_DIRS ".")
