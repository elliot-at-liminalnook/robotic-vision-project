# Build-time guard: fail if managed_components/espressif__esp_video is not the
# repaired copy that patches/esp_video-2.5.0/manifest.json describes (for
# example after managed_components/ was deleted and re-downloaded).
#
# Run by the check_esp_video_fixes target in ../CMakeLists.txt:
#   cmake -DPROJECT_DIR=<project> -P tools/check_esp_video_fixes.cmake
# It runs at build time rather than configure time because the first configure
# is what downloads managed_components/; failing there would be a false alarm.
cmake_minimum_required(VERSION 3.19)  # string(JSON)

set(esp_video_dir "${PROJECT_DIR}/managed_components/espressif__esp_video")
set(how_to_fix "Run:  python3 tools/apply_esp_video_fixes.py  (in ${PROJECT_DIR}), then build again.")

file(READ "${PROJECT_DIR}/patches/esp_video-2.5.0/manifest.json" manifest)
string(JSON count LENGTH "${manifest}")
math(EXPR last "${count} - 1")
foreach(i RANGE ${last})
    string(JSON file GET "${manifest}" ${i} file)
    string(JSON want GET "${manifest}" ${i} after_sha256)
    if(NOT EXISTS "${esp_video_dir}/${file}")
        message(FATAL_ERROR "esp_video fixes: ${esp_video_dir}/${file} is missing.\n${how_to_fix}")
    endif()
    file(SHA256 "${esp_video_dir}/${file}" got)
    if(NOT got STREQUAL want)
        message(FATAL_ERROR
            "esp_video fixes are NOT applied: managed_components/espressif__esp_video/${file} "
            "does not match patches/esp_video-2.5.0/manifest.json.\n"
            "Without them the IMX708 RAW10 stream and flipped Bayer phase are broken.\n${how_to_fix}")
    endif()
endforeach()
