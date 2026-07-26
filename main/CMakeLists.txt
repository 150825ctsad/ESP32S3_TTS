idf_component_register( SRCS 
                        "main.c"

                        INCLUDE_DIRS 
                        "."

                        REQUIRES 
                        relay
                        wifi_cfg
                        TTS
                        sr_ringbuf 
                        player 
                        perf_tester 
                        hardware_driver)
