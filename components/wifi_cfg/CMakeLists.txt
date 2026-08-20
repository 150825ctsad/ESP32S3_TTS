idf_component_register(SRCS "wifi_cfg.c"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES "softap.html" "done.html"
                    EMBED_FILES "${CMAKE_CURRENT_LIST_DIR}/../../assets/wificonfig.wav"
                    PRIV_REQUIRES mqtt_cfg
                    REQUIRES nvs_flash esp_wifi esp_event esp_http_server json ws_cfg
                    )
