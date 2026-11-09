# p4-uvc-camera: ESP32-P4 + Camera Module 3 as a USB webcam

Turns a Waveshare ESP32-P4-WIFI6(-M) + Raspberry Pi Camera Module 3 (IMX708)
into a standard **USB Video Class (UVC) MJPEG webcam**. A Linux PC sees it as
`/dev/videoN` with no drivers or network, so any V4L2 or OpenCV client can open it.

```
IMX708 --MIPI CSI RAW10--> P4 ISP (AE/AWB/AF) --RGB565--> HW JPEG --> USB 2.0 HS (UVC) --> PC
```

**Status (2026-11-09): builds cleanly (ESP-IDF v5.5.5). Not yet flashed or tested on hardware.**
Everything below describes what the code is written to do; none of it has been observed on a board yet.

## What's in here

| Path | What |
|---|---|
| `main/uvc_camera_main.c` | Firmware, adapted from Espressif's esp_video 2.5.0 `uvc` example |
| `main/Kconfig.projbuild` | Image options (`idf.py menuconfig` → "P4 UVC camera"): flips, fixed exposure/gain, fixed focus, JPEG quality |
| `sdkconfig.defaults` | Board, PSRAM, camera, and UVC configuration |
| `patches/esp_video-2.5.0/` | Verified esp_video fixes from `esp32-stereo-camera` (RAW10 receive, Bayer phase after flip, AWB validation) |
| `tools/apply_esp_video_fixes.py` | Applies them to `managed_components/` (checksum-guarded) |
| `tools/check_esp_video_fixes.cmake` | Run on every build; fails it if those fixes are not applied |
| `../components/esp_cam_sensor_imx/` | IMX708 + DW9807 driver: `mushBrainDave/esp32-p4-imx-camera` @ `079e30b` (Apache-2.0) plus local fixes from `esp32-stereo-camera`; see `../components/esp_cam_sensor_imx.local-changes.diff` |

Pinned dependencies (`dependencies.lock`): esp_video 2.5.0, esp_cam_sensor 2.6.0,
esp_ipa 2.4.0, usb_device_uvc 1.2.0~1. These are the same versions as the working Camera Lab firmware.
esp_video and usb_device_uvc are pinned to exact versions in `main/idf_component.yml`: the first because the
patches replace files by checksum, the second because the serial-number code writes to a symbol
(`string_desc_arr`) that is not part of that component's public API.

## Advertised modes

All are IMX708 driver modes, MJPEG. Each is a **centre crop** of the same 2304×1296 binned field, so
smaller sizes see *less of the scene*, not the same scene at lower resolution.

| Size | Field of view |
|---|---|
| 1920×1080 | widest (default) |
| 1280×720 | ~56% of the 1080p width |
| 800×600 | narrow |
| 640×480 | narrowest (~15% of the pixels) |

**Frame rate is unmeasured.** The descriptor advertises 30 fps (UVC needs a number); the sensor modes run at
about 28 fps, and what actually reaches the PC has not been measured, least of all at 1080p. The known limit:
the stream uses a USB isochronous endpoint carrying 1023 bytes per 125 µs microframe, about **8 MB/s**, so
30 fps only fits if the average JPEG stays under roughly 270 KB. The firmware sends one frame at a time and
does not start the next until the last is out, so oversized frames show up as a lower frame rate, not as
corruption. Things to try if 1080p falls short, in this order:

1. Lower `CONFIG_P4CAM_JPEG_QUALITY` (default 80) for smaller frames.
2. Use a smaller mode (1280×720).
3. Switch to bulk transfers (`CONFIG_UVC_MODE_BULK_CAM1=y` instead of `CONFIG_UVC_MODE_ISOC_CAM1`), which are
   not capped per microframe. Untested here, and the idle-stop below then depends on the host no longer
   reading the endpoint.

## Behaviour

- **Serial number:** the USB serial number is the board's base MAC, so two boards get distinct, stable
  `/dev/v4l/by-id/usb-robotic-vision-project_P4_IMX708_Camera_<MAC>-video-index0` names.
- **Start:** every UVC commit (the host selecting a format, which Linux does each time a program starts
  streaming) brings up the whole video stack for the requested size: sensor detect, ISP, JPEG encoder. Expect a
  delay of the order of a second before the first frame. Switching modes is a full teardown and bring-up, the
  same lifecycle Camera Lab uses for mode changes.
- **Stop:** usb_device_uvc has no "stream closed" callback (it only reports USB suspend), so the firmware watches
  for frame requests instead. When the host has not asked for a frame for about **1.5 s**, the sensor, ISP and
  lens motor are shut down (`host stopped reading frames; camera off` in the log). USB suspend stops it
  immediately. If frames are requested again without a new commit, the last mode is brought back up automatically.
- **Recovery:** a capture that yields no frame within 200 ms, or a failed JPEG encode, drops that frame.
  30 dropped frames in a row restart the pipeline (at most once per second).
- **Auto 3A (default):** auto exposure, white balance and autofocus run on the P4 (esp_ipa with the IMX708
  tuning JSON).
- **Image options** (`idf.py menuconfig` → "P4 UVC camera", or `sdkconfig.defaults`; all off by default):

  | Option | Effect |
  |---|---|
  | `P4CAM_HFLIP`, `P4CAM_VFLIP` | Mirror / flip in the sensor (both = 180° rotation). Applied before the sensor mode so the ISP uses the matching Bayer phase. |
  | `P4CAM_FIXED_EXPOSURE` + `P4CAM_EXPOSURE_US`, `P4CAM_GAIN_INDEX` | Turns the auto exposure/gain loop off and sets a fixed exposure time (µs, clamped to the mode's range) and analog gain (driver table index 0-46: 0 = 1.12×, 10 = 2×, 22 = 4×, 34 = 8×, 46 = 16×). |
  | `P4CAM_FIXED_FOCUS` + `P4CAM_FOCUS_POSITION` | Parks the lens at a DW9807 DAC code (0-1023). Only selectable with `CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR=n`: the autofocus loop cannot be paused at run time and would move the lens again. |
  | `P4CAM_JPEG_QUALITY` | JPEG quality 1-100, default 80. |

  Limits, stated plainly:
  - **No fixed white balance.** esp_video 2.5.0 / esp_ipa 2.4.0 have a run-time switch for auto exposure only;
    AWB keeps running and would overwrite any manual gains. Holding it means changing the IMX708 tuning JSON
    (esp_ipa has a fixed-colour-temperature AWB model), which is not done here.
  - Fixed exposure relies on `esp_video_isp_pipeline_set_agc_status()` really stopping exposure/gain updates;
    esp_ipa is a binary library, so that is only verifiable on hardware.
  - These are build-time settings. The UVC descriptor exposes no camera controls, so `v4l2-ctl` on the PC
    cannot change exposure, focus or flips.
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
The build checks this itself: every `idf.py build` compares the three patched files with
`patches/esp_video-2.5.0/manifest.json` and stops with "esp_video fixes are NOT applied" if they differ.
(So a plain `idf.py build` on a fresh checkout downloads the components, fails with that message, and
succeeds after the script has been run.)

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

Any V4L2 client works the same way. OpenCV, for example (request MJPG explicitly, and use the by-id path
so the two boards cannot swap):

```python
import cv2
cap = cv2.VideoCapture("/dev/v4l/by-id/usb-robotic-vision-project_P4_IMX708_Camera_<MAC>-video-index0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
ok, frame = cap.read()   # the first read waits for the camera to start
```

## To verify on hardware

- [ ] Enumerates as UVC on Linux; `/dev/v4l/by-id` names differ per board.
- [ ] Each mode streams; **measure fps per mode** (the `ffmpeg ... -f null -` line above) and note typical JPEG
      size; JPEG integrity (no corruption, check the alignment log line).
- [ ] If 1080p is well under 28 fps: try lower `P4CAM_JPEG_QUALITY`, then bulk mode.
- [ ] Time from opening the device to the first frame; no camera-task stack overflow during bring-up.
- [ ] Idle stop: close the viewer, see `host stopped reading frames; camera off` after ~1.5 s, and the board's
      current draw drop. Re-open: stream comes back.
- [ ] Mode switching: open at 1080p, close, open at 640×480, repeat ~20 times, including re-opening within
      1.5 s of closing (before the idle stop) and after it. No hang, no leak (watch free heap in the log).
- [ ] USB suspend / unplug-replug while streaming: `UVC stop` logged, stream restarts afterwards.
- [ ] Unplug the camera ribbon and open the device: log shows the "Is the camera connected" hint, the host
      gets an error, nothing crashes; works again after re-seating and reboot.
- [ ] Flips: build with `P4CAM_HFLIP`, `P4CAM_VFLIP`, and both; image orientation is right **and colours are
      right** (a wrong Bayer phase shows as a strong colour cast).
- [ ] Fixed exposure: with `P4CAM_FIXED_EXPOSURE`, brightness does not adapt when the scene is covered /
      lit; log shows the values applied. Check white balance still adapts (expected).
- [ ] Fixed focus: with motor control off and `P4CAM_FIXED_FOCUS`, the lens stays put; find the DAC code that
      is sharp at the working distance.
- [ ] Current draw at 1080p with header-only power (USB 2.0 port budget is 500 mA).
- [ ] An OpenCV client (snippet above) reads frames at each size.
