idf_component_register(SRCS "wifi_cfg.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES mqtt_cfg 
                    REQUIRES nvs_flash esp_wifi esp_event
                    )
