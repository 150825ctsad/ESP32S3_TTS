idf_component_register(SRCS "button.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hardware_driver
                    PRIV_REQUIRES esp_driver_gpio
                    )
