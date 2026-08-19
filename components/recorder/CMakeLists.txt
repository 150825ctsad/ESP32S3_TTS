idf_component_register(SRCS "recorder.c"
                    INCLUDE_DIRS "include"
                    REQUIRES hardware_driver mqtt_cfg mbedtls
                    )
