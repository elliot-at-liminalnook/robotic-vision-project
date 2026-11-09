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

# Refuse to build esp_video unless the checksum-guarded repairs in
# patches/esp_video-2.5.0 are applied (tools/apply_esp_video_fixes.py). The
# component manager silently restores the stock files whenever it re-downloads
# managed_components/.
add_custom_target(check_esp_video_fixes
    COMMAND ${CMAKE_COMMAND} -DPROJECT_DIR=${CMAKE_CURRENT_LIST_DIR}
            -P ${CMAKE_CURRENT_LIST_DIR}/tools/check_esp_video_fixes.cmake
    COMMENT "Checking that the esp_video 2.5.0 fixes are applied"
    VERBATIM)
idf_component_get_property(esp_video_lib espressif__esp_video COMPONENT_LIB)
add_dependencies(${esp_video_lib} check_esp_video_fixes)
