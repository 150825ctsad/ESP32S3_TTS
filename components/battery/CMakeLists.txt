idf_component_register(SRCS "battery.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hardware_driver esp_adc
                    PRIV_REQUIRES esp_driver_gpio
                    )
