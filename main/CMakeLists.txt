idf_component_register( SRCS 
                        "main.c"

                        INCLUDE_DIRS 
                        "."

                        REQUIRES 
                        relay
                        button
                        wifi_cfg
                        TTS
                        sr_ringbuf 
                        hardware_driver)
