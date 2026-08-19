idf_component_register(SRCS "ws_cfg.c" "voice_session.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hardware_driver sr_ringbuf esp-sr esp_websocket_client json esp_partition
                    )
