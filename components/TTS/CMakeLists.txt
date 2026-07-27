idf_component_register(SRCS "TTS.c"
                    INCLUDE_DIRS "include"
                    REQUIRES sr_ringbuf
                             esp-sr
                             hardware_driver
                             esp_partition
                             driver)
