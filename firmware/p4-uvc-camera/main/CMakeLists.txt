idf_component_register(
    SRCS "uvc_camera_main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx usb_device_uvc espressif__tinyusb esp_timer esp_mm esp_hw_support
)

# Fail the build if the IMX708 ISP tuning file didn't resolve (esp_ipa only warns).
idf_build_get_property(imx708_ipa_json IMX708_IPA_JSON_RESOLVED)
if(NOT EXISTS "${imx708_ipa_json}")
    message(FATAL_ERROR "IMX708 ISP tuning config not found at ${imx708_ipa_json}")
endif()
