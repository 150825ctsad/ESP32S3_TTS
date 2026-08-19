idf_component_register( SRCS
                        "main.c"

                        INCLUDE_DIRS
                        "."

                        REQUIRES
                        button
                        relay
                        wifi_cfg
                        ws_cfg
                        sr_ringbuf
                        hardware_driver)
