# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(TTS)

# 将 TTS 语音数据镜像注册进 idf.py flash 烧录列表（写入 voice_data 分区，见 partitions.csv）
 set(voice_data_image ${PROJECT_DIR}/managed_components/espressif__esp-sr/esp-tts/esp_tts_chinese/esp_tts_voice_data_xiaoxin_custom.dat)
 esptool_py_flash_to_partition(flash "voice_data" "${voice_data_image}")
