# Build plan

Decisions as of 2026-11-09. Update this file when a decision changes. The reasons are in `DECISIONS.md`.

## Decisions

| Area | Decision |
|---|---|
| Arms | Fully custom follower, printed on our own printer (Bambu). Upstream TheRobotStudio and Hiwonder files are the starting point only. **No leader arm** (decided 2026-11-10): teleop is keyboard/mouse. |
| Servos | Hiwonder **HX-30HM** bus servos on the follower (ordered 2026-11-10, see `BOM.md`; no HX-10HM). The packet protocol is Feetech-STS-compatible, but the **register map must be verified before any write** (DECISIONS P5; see `third_party/hiwonder-so-arm101/README.md`). |
| Servo driver | Hiwonder **BusLinker V3.0** (chosen 2026-11-10: same store as the servos). USB to half-duplex TTL at 1 Mbps. |
| Servo power | **12.0 V bench supply, ~20 A** (decided, DECISIONS F1). HX servos: rated 11.1 V, range 9–12.6 V; Hiwonder kit ships 12 V 5 A. Waveshare adapter passes input through unregulated (wiki: 9–12.6 V input). Stall is ~3.2 A per servo at 12 V. |
| Cameras | Raspberry Pi **Camera Module 3 Wide** (IMX708): **one on the follower's wrist + one fixed scene camera**, optionally a second fixed camera from another angle (decided 2026-11-08). |
| Camera boards | 2× Waveshare **ESP32-P4-WIFI6-M**, one per camera. |
| Camera transport | **USB UVC webcam** (decided 2026-11-08): new firmware `firmware/p4-uvc-camera/`, so each board appears to the Linux PC as a standard webcam. Wi-Fi Camera Lab firmware remains the fallback. |
| Compute host | Linux PC with GPU (teleop, recording, training, inference, simulation). |
| Learning stack | **Not LeRobot** (decided 2026-11-08). Simulation in **Newton 1.7 with `SolverMuJoCo` on the MuJoCo Warp backend**; see `sim/`. |
| Simulation | `sim/`: generated from upstream `Simulation/SO101` with fixed jaw collision, finger pads, HX-30HM torque limit, 3 cameras, workcell. Runs on Newton (Linux) and plain MuJoCo (anywhere). |

## Parts ordered

See `BOM.md` (links, prices, what each part is for, power wiring, what is still to buy locally).

## Parts on hand

| Qty | Item | Role |
|---:|---|---|
| 2 | Raspberry Pi Camera Module 3 Wide | One on the wrist, one fixed scene camera |
| 2 | ESP32-P4-WIFI6-M dev board | Camera host: CSI → ISP → HW JPEG → USB UVC (Wi-Fi MJPEG is the fallback) |
| 2 | Pi Zero v1.3 camera cable, 150 mm | Camera ↔ board (the board sits right behind the camera on both mounts, so 150 mm is more than enough; folding it is undecided) |
| 2 | Pro Mini camera mount for Pi Zero | Possibly reusable; dedicated printed mounts are likely better |
| 2 | USB-C ↔ USB-C, USB 3.2 Gen 2, 3 ft | Board power / flashing / serial console |

## Camera pipeline (existing Wi-Fi firmware, proven)

Prior work in `~/Projects/esp32-stereo-camera` (see its `outputs/board-diagnostics/CAMERA-TECHNICAL-REFERENCE.md`):

- **Driver:** `mushBrainDave/esp32-p4-imx-camera` @ `079e30b` (Apache-2.0), ESP-IDF v5.5.5,
  esp_video 2.5.0 / esp_cam_sensor 2.6.0. IMX708 over 2-lane MIPI CSI, RAW10, P4 ISP → RGB565,
  hardware JPEG. DW9807 lens motor at I²C `0x0c`, sensor at `0x1a`.
- **Firmware ("Camera Lab"):** HTTP control on `:80`, MJPEG stream on `:81/stream.mjpg` (3 viewer slots),
  USB serial control at 2 Mbaud. Wi-Fi via the on-board ESP32-C6. Both boards were already flashed and streaming (2026-10-03).
- **Measured Wi-Fi delivery** (one board, short bench samples): 640×480 ≈ 27 fps, 800×600 ≈ 27 fps,
  1024×768 ≈ 15 fps, 1280×720 ≈ 5 fps, 1080p ≈ 9 fps.
  640×480 @ ~27–30 fps is a common setup for imitation-learning datasets.
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

## Camera input options (history)

We don't use LeRobot (DECISIONS H5), but this comparison is why the UVC firmware exists. A generic OpenCV
client has the same problems with the Wi-Fi MJPEG streams as LeRobot's camera class did (checked against
`huggingface/lerobot@d490e47`):
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
   (compiles; not yet flashed or tested; first-flash risks from the 2026-11-09 audit are fixed, DECISIONS U1–U5). Background:
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
   - **One board per camera** (P4 has one CSI). The firmware sets the USB serial number from the board's MAC at
     boot, so each board gets a stable `/dev/v4l/by-id` name with no per-board build.
   - **Advantages over Wi-Fi:** any V4L2 / OpenCV client with no custom code, deterministic bandwidth,
     and full field of view at 1080p if the rate holds (unmeasured: the isochronous endpoint carries about
     8 MB/s, so 1080p may land at 12–20 fps; bulk mode and JPEG quality are the knobs).

## Open items

- [ ] Make the MX1.25-4P → USB-A male cable (pinout in `firmware/p4-uvc-camera/README.md`), flash, verify UVC on Linux.
- [ ] Lock exposure / white balance / focus for dataset recording. The firmware now has build-time options for fixed exposure + gain, fixed focus (with autofocus compiled out) and h/v flip; **fixed white balance is not possible** with esp_video 2.5.0 without changing the IMX708 tuning file. Test them on hardware.

- [x] HX-10HM / HX-30HM docs, register table, servo STEP: `docs/datasheets/hiwonder-hx-servos/`.
- [ ] Print upstream `STL/Gauges/Gauge_0.STL` and test-fit an HX servo. If it fits, upstream SO-101 STLs are usable as-is.
- [x] Sim actuator params for HX servos: realistic position loop from a 9° prior (DECISIONS P1). Still to do: replace the prior with a bench measurement (below).
- [ ] Identify Hiwonder 3MF/3DM parts (names are generic `Document_N`) and map them to SO-101 links.
- [ ] Fixed camera mount design (camera + ESP32-P4 board, 150 mm ribbon). Reuse ideas from the
      stereo enclosure prototypes in `~/Projects/esp32-stereo-camera/outputs/stereo-prototype-v*`.
- [ ] Camera intrinsics/extrinsics calibration for each fixed view.
- [x] Sim cameras (wrist + 2 fixed) in `sim/config/cameras.toml`.
- [ ] Replace the placeholder intrinsics and fixed-camera poses in `cameras.toml` with measured ones.
- [ ] Verify the sim on the Linux GPU PC with `--solver warp` (MuJoCo Warp); only the CPU path has been run so far. Run `pytest sim/tests` there too, and check that Newton honours the wrist-mount contact pairs (DECISIONS A6).
- [x] Wrist camera mount designed (`hardware/wrist_mount`): bracket on the gripper's +y side, board on the holder's back; pose feeds the sim.
- [ ] Print + test: camera holder snap fit, bracket fit on the gripper, wobble; design the ESP32-P4 cradle on the holder's back.
- [ ] Fixed scene-camera stand for the holder (2× M3 holes), then measure its pose into `cameras.toml`.
- [x] Servo bus in the loop (2026-11-11): byte-level bus, emulated HX-30HM servos, the controller's driver; `teleop.py --bus` (DECISIONS X1–X12, `sim/README.md`).
- [ ] Real-robot backend for `teleop.py`: the bus half exists (`so101sim/bus/driver.py`, `arm.py`); still needed are a serial `Port` (pyserial), a real-time `step()`, and UVC cameras. The camera-safe wrist-roll range (−2.34…+0.80 rad) was to go into servo 5's angle-limit registers, but **the HX register table lists none (9–12)**: read those addresses on a real servo; if they don't exist, the limit stays in software only.
- [ ] On the real bus, measure the `servos.toml [bus]` placeholders (servo response delay, USB latency, whether the BusLinker echoes) and check the emulator's assumptions listed at the top of `so101sim/bus/servo_emu.py` (goal write vs torque switch, speed 0 = no limit, out-of-range writes, position calibration).
- [x] Model audit fixes, command filter, physics monitor (2026-11-11): DECISIONS M1–M10, V1–V10; `sim/README.md`, "Physical invariants".
- [x] Wires in the model (2026-11-11): `teleop.py --cables`; DECISIONS K1–K8.
- [ ] Wrist camera ribbon: decide how its spare 100 mm is folded (unfolded, it hangs to 13 mm above the table at a grasp), and tie the fixed cameras' USB cables down behind the camera (DECISIONS K11).
- [ ] Wires: measure each wire's weight per metre and bending stiffness (sag test in `sim/config/cables.toml`), find where the USB header and the driver board's bus socket really are, and decide how the four leads inside the brackets are routed.
- [ ] Camera cable: the spliced lead (about 0.45 m) is shorter than the 0.76 m the cable needs on the arm (`BOM.md`, "Not checked").
- [ ] Print check: the camera bracket clears the wrist link by only 0.7 to 2 mm over half the roll range (DECISIONS M1), and the base is assumed to rest on its lowest faces (M4).
- [ ] Command filter and wires on the Newton backend: the filter is wired in but has never run there; the wires are MuJoCo only.
- [ ] Teleop: make one large jump of the Cartesian target a straight-line move too (DECISIONS V10).
- [x] Stop path (2026-11-14): freeze (`X`), torque off (`T` and on every exit), confirmed over the bus (DECISIONS E1–E11).
- [ ] Buy and fit a switch on the 12 V supply lead: the stop that works with the computer or the bus dead (DECISIONS E2).
- [ ] Stop path follow-ups: `torque_off()` / `torque_on()` on the Newton backend; torque off at the end of `pick_demo.py`; a watchdog that freezes or switches torque off on servo feedback (temperature, voltage, status bits, replies lost in a row).
- [ ] On a real HX-30HM: does a goal write switch torque on, and does a servo with torque off coast or brake (DECISIONS E5, E9)?
- [x] Bench scene and task with a reference solution through the API (2026-11-11): DECISIONS T5–T11.
- [x] REST API for a language model (2026-11-11): `sim/scripts/sim_api.py`; DECISIONS R1–R8. Retired 2026-11-15 in favour of the hub's API (`hub/`, `sim/scripts/run_system.py`; DECISIONS MD118).
- [ ] Try the API with a model end to end (nothing has driven it yet except tests and curl), with and without object positions; add what it turns out to need (likely: a way to point at a pixel in a camera picture).
- [x] Camera firmware in the loop (2026-11-11): `p4-uvc-camera` built for the PC with stand-in sensor, encoder, FreeRTOS and USB; `teleop.py --camera-fw` (DECISIONS X13–X22, `firmware/p4-uvc-camera/host/README.md`).
- [ ] Rebuild the firmware with ESP-IDF after the `s_last_fb_get` change (X20).
- [ ] On the board, measure the stand-in timings (`host/README.md`: bring-up, readout, encode time per megapixel, real JPEG sizes and frame rate per size) and put them into the PC build.
- [ ] Scene cameras through the firmware too (needs click-to-move to work on a cropped or delayed view), and a controller-side camera reader shared with the real robot (V4L2/OpenCV underneath).
- [ ] Servo bus in the loop on the Newton backend (MuJoCo only today).
- [ ] **Before any register writes (incl. stock LeRobot `connect()`):** compare a real HX-30HM with the register table. The table V3.7 agrees with the STS3215 layout (the conflict reported earlier was a misreading, DECISIONS X10), but it has not been checked against a servo. Read model and firmware, dump registers, test with a harmless write. `BusArm.connect()` does the read-only part and refuses to write if the registers don't look right.
- [ ] On the real HX-30HM: measure position error under a known load (stiffness), a step response (D term, loop ringing frequency), whether overload protection is relative to the torque-limit register, and temperature after a 10-min far-reach hold; update `ServoSpec` (`sat_error_deg`, `d_time_s`) and the governor's `bandwidth_rad_s`, rerun `servo_load_study.py` and `fast_teleop_study.py`.
- [ ] Real-robot teleop: stream governor commands as goal positions (servo goal speed = max, acceleration off), feed measured positions back for the lag guard and path pacing, log present current (register 69) against the governor's predicted torque. Gripper torque limit 500.
- [x] Power supply: 12.0 V bench supply sized for ~20 A (DECISIONS F1).
- [x] Fast teleop: torque governor, straight-line clicks, load-aware IK (default 0°): `sim/docs/FAST_TELEOP_FINDINGS.md`.
- [x] Realistic servo position loop in the sim + gravity/back-EMF feed-forward + grip-by-offset (`so101sim/command.py`, DECISIONS P1–P4).
- [ ] Policy training: research notes, recipe and phased plan in `docs/POLICY_TRAINING_IDEAS.md`. Next: benchmark the MuJoCo Warp renderer on the GPU PC, pick mjlab vs Newton for RL, build the state-based pick teacher.
- [ ] Rerun `servo_load_study.py` and `fast_teleop_study.py` with the current servo loop (9° stiffness, feed-forward, 12 V, gripper at 50 %): their published numbers predate it (see the notes at the top of the two FINDINGS files).

## Open decisions

Found by the 2026-11-09 audit. None blocks the sim work; each blocks the step named.

| Decision | Options | Blocks |
|---|---|---|
| Wrist camera mode | 1080p downscaled on the host (full field of view, the sim's assumption, probably 12–20 fps) or a cropped mode at ~30 fps (then change the sim intrinsics) | Camera calibration, policy observations |
| Exposure / focus lock | Fixed at build time (available now) or UVC controls from the host (needs descriptor work); white balance needs a tuning-file change either way | Dataset recording |
| RL framework | mjlab or Newton, after the GPU renderer benchmark | Policy training |
| Servo control library | Own minimal Feetech-protocol driver, or a patched existing one, once the register map is verified | Real-robot backend |
| Jaw pads | Add rubber/TPU pads to the real jaws (the sim assumes friction 1.0) or lower the sim's pad friction to bare PLA (~0.35) | Sim-to-real grasping |
| ESP32-P4 cradle and cable routing | Cradle on the holder's back, ribbon fold. A USB cable path down the arm is now modelled (DECISIONS K6, K7: clips on the forearm and upper arm, first stretch 260 mm); confirm or change it | Printing the wrist mount |
| Fixed scene-camera stand | Design, then measure its pose | Scene camera calibration |
| Upstream STLs with HX servos | Print `Gauge_0.STL` and test-fit; if it fits, use upstream STLs unchanged | Printing the arm |
