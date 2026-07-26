idf_component_register( SRCS 
                        "main.c"

                        INCLUDE_DIRS 
                        "."

                        REQUIRES 
                        TTS
                        sr_ringbuf 
                        player 
                        perf_tester 
                        hardware_driver)
