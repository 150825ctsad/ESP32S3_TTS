idf_component_register(SRCS "mqtt_cfg.c"
                    INCLUDE_DIRS "include"
                    REQUIRES mqtt json esp_hw_support esp_wifi esp_netif hardware_driver ws_cfg battery
                    )
