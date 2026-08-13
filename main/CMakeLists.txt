idf_component_register( SRCS 
                        "main.c"

                        INCLUDE_DIRS 
                        "."

                        REQUIRES 
                        button
                        relay
                        wifi_cfg
                        TTS
                        sr_ringbuf 
                        hardware_driver)
