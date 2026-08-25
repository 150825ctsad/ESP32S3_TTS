idf_component_register(SRCS "led_status.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hardware_driver wifi_cfg mqtt_cfg esp_wifi
                    PRIV_REQUIRES esp_driver_gpio
                    )
