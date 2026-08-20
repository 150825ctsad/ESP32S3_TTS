# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TTS)

# 将提示音 WAV 烧录进 voice_data 分区（16kHz/16bit/mono，见 partitions.csv）
set(voice_data_image ${PROJECT_DIR}/assets/welcome.wav)
esptool_py_flash_to_partition(flash "voice_data" "${voice_data_image}")

# 离线中文 TTS 语音库（xiaoxin_small ≈ 2.8MB，放入 tts_data 分区）
set(tts_voice_image ${PROJECT_DIR}/managed_components/espressif__esp-sr/esp-tts/esp_tts_chinese/esp_tts_voice_data_xiaoxin_small.dat)
esptool_py_flash_to_partition(flash "tts_data" "${tts_voice_image}")
