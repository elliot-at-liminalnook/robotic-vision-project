# p4-uvc-camera: ESP32-P4 + Camera Module 3 as a USB webcam

Turns a Waveshare ESP32-P4-WIFI6(-M) + Raspberry Pi Camera Module 3 (IMX708)
into a standard **USB Video Class (UVC) MJPEG webcam**. A Linux PC sees it as
`/dev/videoN` with no drivers or network, and LeRobot's stock `OpenCVCamera` can open it.

```
IMX708 --MIPI CSI RAW10--> P4 ISP (AE/AWB/AF) --RGB565--> HW JPEG --> USB 2.0 HS (UVC) --> PC
```

**Status (2026-11-08): builds cleanly (ESP-IDF v5.5.5). Not yet flashed or tested on hardware.**

## What's in here

| Path | What |
|---|---|
| `main/uvc_camera_main.c` | Firmware, adapted from Espressif's esp_video 2.5.0 `uvc` example |
| `sdkconfig.defaults` | Board, PSRAM, camera, and UVC configuration |
| `patches/esp_video-2.5.0/` | Verified esp_video fixes from `esp32-stereo-camera` (RAW10 receive, Bayer phase after flip, AWB validation) |
| `tools/apply_esp_video_fixes.py` | Applies them to `managed_components/` (checksum-guarded) |
| `../components/esp_cam_sensor_imx/` | IMX708 + DW9807 driver: `mushBrainDave/esp32-p4-imx-camera` @ `079e30b` (Apache-2.0) plus local fixes from `esp32-stereo-camera`; see `../components/esp_cam_sensor_imx.local-changes.diff` |

Pinned dependencies (`dependencies.lock`): esp_video 2.5.0, esp_cam_sensor 2.6.0,
esp_ipa 2.4.0, usb_device_uvc 1.2.0. These are the same versions as the working Camera Lab firmware.

## Advertised modes

All are IMX708 driver modes, MJPEG, nominal 30 fps (sensor delivers ~28 fps).
Each is a **centre crop** of the same 2304×1296 binned field, so smaller sizes
see *less of the scene*, not the same scene at lower resolution.

| Size | Field of view |
|---|---|
| 1920×1080 | widest (default) |
| 1280×720 | ~56% of the 1080p width |
| 800×600 | narrow |
| 640×480 | narrowest (~15% of the pixels) |

## Behaviour

- **Serial number:** the USB serial number is the board's base MAC, so two boards get distinct, stable
  `/dev/v4l/by-id/usb-robotic-vision-project_P4_IMX708_Camera_<MAC>-video-index0` names.
- **Start/stop:** each time the host opens the camera, the firmware brings up the whole video stack for the requested
  size and tears it down on close. This is the same lifecycle Camera Lab uses for mode changes.
- **Auto 3A:** auto exposure, white balance and autofocus run on the P4 (esp_ipa with the IMX708 tuning JSON).
  For dataset recording we'll want them locked; not implemented yet.
- **Alignment guard:** if the capture buffers aren't 128-byte aligned, frames are copied to an aligned buffer
  before JPEG (misaligned input gave corrupt JPEGs on this board). The log says when this happens.
- **Console:** logs go to the **USB-C** port (CH343 serial bridge) at 115200 baud.

## Cable

The webcam is on the P4's native USB, which is the **4-pin MX1.25 header marked "USB"**, not the
USB-C port. Header pinout (schematic `P1`): **1 = 5V (`VCC_5V`), 2 = D−, 3 = D+, 4 = GND**.

Make or buy an MX1.25 4-pin → USB-A (or USB-C) **male** cable:

| Header pin | USB-A pin | Wire |
|---|---|---|
| 1 `VCC_5V` | 1 VBUS | red (see note) |
| 2 `D−` | 2 D− | white |
| 3 `D+` | 3 D+ | green |
| 4 GND | 4 GND | black |

**Power note:** header pin 1 connects straight to the board's 5V rail with no diode.
- **Single cable (deployment):** connect all 4 wires; the PC powers the board.
- **Development (USB-C also plugged in for logs/flashing):** leave pin 1 **unconnected**. Otherwise USB-C's 5V
  can push back into the PC's USB port.

Keep the D+/D− pair twisted and short; it's a 480 Mbit/s link.

## Build

```bash
. ~/esp/esp-idf/export.sh            # ESP-IDF v5.5.5
cd firmware/p4-uvc-camera
idf.py reconfigure                   # downloads managed_components/
python3 tools/apply_esp_video_fixes.py
idf.py build
```

Re-run `apply_esp_video_fixes.py` whenever `managed_components/` is re-downloaded (e.g. after deleting it).

## Flash (keeps Camera Lab's partition table and Wi-Fi settings)

Both boards currently run Camera Lab, whose partition table has the app at `0x10000` (4 MB) and NVS
(Wi-Fi credentials) at `0x9000`. This firmware is ~570 KB, so **flash only the app**:

```bash
idf.py -p /dev/cu.usbmodemXXXX app-flash monitor   # USB-C port
```

Stop any Camera Lab USB bridge (`Start Camera.command`) first; it holds the serial port.

**Roll back to Camera Lab:**

```bash
python -m esptool --chip esp32p4 -p /dev/cu.usbmodemXXXX write_flash 0x10000 \
  ~/Projects/esp32-stereo-camera/outputs/camera-web/camera-web.bin
```

(Board A also has a first-1 MB factory backup, and board B a first-2 MB backup, in
`~/Projects/esp32-stereo-camera`; see that project's notes.)

## Test on Linux

```bash
lsusb | grep 303a                         # Espressif VID 0x303A, PID 0x8000
ls -l /dev/v4l/by-id/                     # one entry per board, named by MAC
v4l2-ctl -d /dev/video0 --list-formats-ext  # MJPG 1920x1080, 1280x720, 800x600, 640x480
ffplay -f v4l2 -input_format mjpeg -video_size 1920x1080 /dev/video0
ffmpeg -f v4l2 -input_format mjpeg -video_size 1920x1080 -i /dev/video0 -t 10 -f null -  # measure fps
```

LeRobot (once verified):

```python
from lerobot.cameras.opencv import OpenCVCameraConfig
cam = OpenCVCameraConfig(index_or_path="/dev/v4l/by-id/usb-robotic-vision-project_P4_IMX708_Camera_<MAC>-video-index0",
                         fps=30, width=1920, height=1080, fourcc="MJPG")
```

## To verify on hardware

- [ ] Enumerates as UVC on Linux; `/dev/v4l/by-id` names differ per board.
- [ ] Each mode streams; measured fps per mode; JPEG integrity (no corruption, check alignment log line).
- [ ] Start/stop/switch modes repeatedly (full stack teardown each time).
- [ ] Current draw at 1080p with header-only power (USB 2.0 port budget is 500 mA).
- [ ] LeRobot `OpenCVCamera` connects (its FPS check passes with the advertised 30 fps).
