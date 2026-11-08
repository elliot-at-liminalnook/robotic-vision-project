# Build plan

Decisions as of 2026-11-08. Update this file when a decision changes.

## Decisions

| Area | Decision |
|---|---|
| Arms | Fully custom leader + follower, printed on our own printer (Bambu). Upstream TheRobotStudio and Hiwonder files are the starting point only. |
| Servos | Hiwonder **HX-10HM / HX-30HM** bus servos (to buy). Feetech-STS-compatible protocol, so stock LeRobot `sts3215` drives them (see `third_party/hiwonder-so-arm101/README.md`). |
| Servo driver | Hiwonder BusLinker V3.0 or Waveshare Bus Servo Adapter (A). Either one is USB to half-duplex TTL at 1 Mbps. |
| Servo power | **12V supply** (HX servos: rated 11.1V, range 9–12.6V; Hiwonder kit ships 12V 5A). Waveshare adapter passes input through unregulated (wiki: 9–12.6V input). Stall is 3A per servo, so size the supply for simultaneous moves. |
| Cameras | Raspberry Pi **Camera Module 3 Wide** (IMX708): **one on the follower's wrist + one fixed scene camera**, optionally a second fixed camera from another angle (decided 2026-11-08). |
| Camera boards | 2× Waveshare **ESP32-P4-WIFI6-M**, one per camera. |
| Camera transport | **USB UVC webcam** (decided 2026-11-08): new firmware `firmware/p4-uvc-camera/`, so each board appears to the Linux PC as a standard webcam. Wi-Fi Camera Lab firmware remains the fallback. |
| Compute host | Linux PC with GPU (teleop, recording, training, inference, simulation). |
| Learning stack | **Not LeRobot** (decided 2026-11-08). Simulation in **Newton 1.7 with `SolverMuJoCo` on the MuJoCo Warp backend**; see `sim/`. |
| Simulation | `sim/`: generated from upstream `Simulation/SO101` with fixed jaw collision, finger pads, HX-30HM torque limit, 3 cameras, workcell. Runs on Newton (Linux) and plain MuJoCo (anywhere). |

## Parts on hand

| Qty | Item | Role |
|---:|---|---|
| 2 | Raspberry Pi Camera Module 3 Wide | Fixed scene cameras |
| 2 | ESP32-P4-WIFI6-M dev board | Camera host: CSI → ISP → HW JPEG → Wi-Fi MJPEG |
| 2 | Pi Zero v1.3 camera cable, 150 mm | Camera ↔ board (fixed mounts, so 150 mm is enough if the board sits behind the camera) |
| 2 | Pro Mini camera mount for Pi Zero | Possibly reusable; dedicated printed mounts are likely better |
| 2 | USB-C ↔ USB-C, USB 3.2 Gen 2, 3 ft | Board power / flashing / serial console |

## Camera pipeline (existing, proven)

Prior work in `~/Projects/esp32-stereo-camera` (see its `outputs/board-diagnostics/CAMERA-TECHNICAL-REFERENCE.md`):

- **Driver:** `mushBrainDave/esp32-p4-imx-camera` @ `079e30b` (Apache-2.0), ESP-IDF v5.5.5,
  esp_video 2.5.0 / esp_cam_sensor 2.6.0. IMX708 over 2-lane MIPI CSI, RAW10, P4 ISP → RGB565,
  hardware JPEG. DW9807 lens motor at I²C `0x0c`, sensor at `0x1a`.
- **Firmware ("Camera Lab"):** HTTP control on `:80`, MJPEG stream on `:81/stream.mjpg` (3 viewer slots),
  USB serial control at 2 Mbaud. Wi-Fi via the on-board ESP32-C6. Both boards were already flashed and streaming (2026-10-03).
- **Measured Wi-Fi delivery** (one board, short bench samples): 640×480 ≈ 27 fps, 800×600 ≈ 27 fps,
  1024×768 ≈ 15 fps, 1280×720 ≈ 5 fps, 1080p ≈ 9 fps.
  **640×480 @ ~27–30 fps matches LeRobot's typical camera setup.**
- **Smaller modes are center crops** of a 2304×1296 binned field, not downscales. So 640×480
  sees much less of the scene than 1080p, and the "wide" lens doesn't give a wide view at VGA.
  This matters for camera placement.

Oddities already resolved or documented there:
- **RAW10 stall:** fixed by clearing ISP line-start/end expectations (IMX708 + bypass only).
- **Pink cast:** removed by replacing the provisional color matrix with identity. A green cast remains (no calibrated CCM).
- **IPA (auto 3A) disabled** in Camera Lab. Auto exposure, white balance and focus run host-side (bridge or browser).
  **For dataset recording, use fixed exposure / white balance / focus** so frames stay consistent.
- **`VIDIOC_G_FMT` reports `sizeimage = 0`:** derive buffer sizes from width × height instead.
- **No frame timestamps or sensor-level sync between boards.** The two views are not exposure-synchronized.

## LeRobot integration: camera input

Stock LeRobot can't use these streams directly (checked against `huggingface/lerobot@d490e47`):
- `OpenCVCameraConfig.index_or_path` is `int | Path`. From the CLI, a URL becomes a `Path`, and
  `str(Path("http://h:81/stream.mjpg"))` == `"http:/h:81/stream.mjpg"`, which is broken.
- On connect it calls `VideoCapture.set(CAP_PROP_FPS)` and validates the result. Network streams generally can't honor that.

Options:
1. **Custom LeRobot camera class** (fallback if UVC fails): `ESP32MjpegCameraConfig` registered as a
   `CameraConfig` subclass. It reads the HTTP MJPEG stream in a background thread, keeps the latest frame,
   timestamps on receipt, reconnects, and pushes fixed exposure/focus via the `:80` API on connect.
2. **Virtual webcam on the Linux host:** `ffmpeg` → `v4l2loopback` `/dev/videoN`, then stock OpenCVCamera
   with `fps` unset. No LeRobot code, but more moving parts and the FPS check still needs care.
3. **USB UVC webcam firmware** on the P4 (wired, no Wi-Fi). **Chosen.** Built in `firmware/p4-uvc-camera/`
   (compiles; not yet flashed or tested). Background:
   - **Software:** Espressif's `esp_video` `examples/uvc` (create it with `idf.py create-project-from-example "espressif/esp_video:uvc"`) on top of esp-iot-solution `usb_device_uvc` (TinyUSB, defaults to the P4's HS port). Supports MJPEG or H.264; use MJPEG for OpenCV.
   - **Defaults:** the example's default is SC2336 @ 720p30, so it needs porting to our IMX708 driver and RAW10 fix.
   - **Expected rate:** hardware allows up to 1080p (JPEG 1080p@40). 1080p MJPEG over USB 2.0 HS at ~25–30 fps is an estimate, not measured.
   - **Cabling catch:** the board's USB-C port is a CH343P serial bridge, not the P4's USB. UVC needs the
     4-pin MX1.25 "USB" header wired to a USB male plug. **Our USB-C cables can't carry it as-is.**
   - **Power over the header** (schematic page 1, verified 2026-11-08):
     - The `P1` "MX1.25 4pin" header is 1 = `VCC_5V`, 2 = `USBD_N`, 3 = `USBD_P`, 4 = GND.
     - `VCC_5V` is the main 5V rail; it feeds the MP1658 3.3V buck. So **a single header-to-PC cable powers the board.**
     - The USB-C `USB0_5V` reaches `VCC_5V` through an ideal-diode circuit (Q2 AO3401 P-FET + U2 MMDT3906 mirror),
       so nothing flows back out of the USB-C port.
     - The header has **no** such protection. With both USB-C and the header plugged in, USB-C's 5V can
       push into the PC port's VBUS. **For development with both connected, wire only D−/D+/GND on the
       header cable (leave pin 1 open)** and power from USB-C.
     - Current draw at full camera + JPEG load hasn't been measured. Check it against a USB 2.0 port's 500 mA budget.
   - **One board per camera** (P4 has one CSI). Set a unique `CONFIG_TUSB_SERIAL_NUM` per board.
     Both default to VID 0x303A / PID 0x8000 / serial "12345678"; unique serials give stable `/dev/v4l/by-id` names.
   - **Advantages over Wi-Fi:** stock LeRobot `OpenCVCamera` with no custom code, deterministic bandwidth,
     and full field of view at 1080p if the rate holds.

## Open items

- [ ] Make the MX1.25-4P → USB-A male cable (pinout in `firmware/p4-uvc-camera/README.md`), flash, verify UVC on Linux.
- [ ] Lock exposure / white balance / focus for dataset recording (UVC controls or fixed firmware settings).

- [x] HX-10HM / HX-30HM docs, register table, servo STEP: `docs/datasheets/hiwonder-hx-servos/`.
- [ ] Print upstream `STL/Gauges/Gauge_0.STL` and test-fit an HX servo. If it fits, upstream SO-101 STLs are usable as-is.
- [ ] Sim actuator params for HX servos (upstream MJCF uses STS3215 values from Open Duck Mini).
- [ ] Identify Hiwonder 3MF/3DM parts (names are generic `Document_N`) and map them to SO-101 links.
- [ ] Fixed camera mount design (camera + ESP32-P4 board, 150 mm ribbon). Reuse ideas from the
      stereo enclosure prototypes in `~/Projects/esp32-stereo-camera/outputs/stereo-prototype-v*`.
- [ ] Camera intrinsics/extrinsics calibration for each fixed view.
- [x] Sim cameras (wrist + 2 fixed) in `sim/config/cameras.toml`; replace placeholder poses/intrinsics with measured ones.
- [ ] Verify the sim on the Linux GPU PC with `--solver warp` (MuJoCo Warp); only the CPU path has been run so far.
- [x] Wrist camera mount designed (`hardware/wrist_mount`): bracket on the gripper's +y side, board on the holder's back; pose feeds the sim.
- [ ] Print + test: camera holder snap fit, bracket fit on the gripper, wobble; design the ESP32-P4 cradle on the holder's back.
- [ ] Fixed scene-camera stand for the holder (2× M3 holes), then measure its pose into `cameras.toml`.
- [ ] Real-robot backend for `teleop.py` (HX servos over the bus, `ServoMap` ticks↔rad, UVC cameras).
- [ ] On the real HX-30HM: measure position error under a known load (stiffness), check whether overload protection is relative to the torque-limit register, and log temperature after a 10-min far-reach hold; update `ServoSpec` and rerun `servo_load_study.py`.
- [ ] Set servo registers: goal speed/acceleration to match the gentle profile; gripper torque limit 500.
- [ ] Power supply: ≥ 10 A at 12 V if full-speed motion is ever wanted (the kit's 5 A is fine for the gentle profile).
- [ ] Power: servo supply voltage for HX servos; BusLinker vs Waveshare board.
