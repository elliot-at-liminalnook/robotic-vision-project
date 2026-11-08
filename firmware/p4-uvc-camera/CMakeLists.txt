# ESP32-P4 + Raspberry Pi Camera Module 3 (IMX708) as a USB UVC webcam.
# Based on Espressif's esp_video 2.5.0 "uvc" example, adapted to the IMX708
# driver in ../components/esp_cam_sensor_imx.
cmake_minimum_required(VERSION 3.16)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# IMX708 ISP tuning (AE/AWB/AF/CCM). esp_ipa reads this property while its own
# CMakeLists is processed, so it must be set between project.cmake and project().
# A wrong path is NOT an error in esp_ipa (it prints and continues, giving a
# stream with no auto-exposure or white balance); main/CMakeLists.txt re-checks.
set(imx708_ipa_json "${CMAKE_CURRENT_LIST_DIR}/../components/esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json")
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH "${imx708_ipa_json}" APPEND)
idf_build_set_property(IMX708_IPA_JSON_RESOLVED "${imx708_ipa_json}")

project(p4_uvc_camera)
