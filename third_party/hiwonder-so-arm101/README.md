# Hiwonder SO-ARM101 (HX-servo variant) — vendor files

Downloaded 2026-11-08 from Hiwonder's official "Download All" Google Drive folder
(linked from https://www.hiwonder.com.cn/store/learn/185.html):
https://drive.google.com/drive/folders/17QlQJu2PsdpZemjYbUUrwbZgVUeDqA-D

No license is stated for these files. They are Hiwonder's published support
material; treat them as reference only (not Apache-2.0 like `../SO-ARM100`).

**Do not edit files here.** Copy into `hardware/` to modify.

## How this differs from the standard SO-101

Hiwonder's kit replaces the Feetech STS3215 servos with its own
**HX-10HM** (×6) and **HX-30HM** (×6) magnetic-encoder bus servos and uses a
**BusLinker V3.0** debug board. Findings from the files below:

- **Protocol-compatible with Feetech STS.** `manuals/HX-30HM_Communication_Protocol_v1.0.pdf`
  describes the same `0xFF 0xFF | ID | Len | Instr | Params | ~Checksum` packet format at
  1 Mbps, 8N1, IDs 0–253, broadcast 254, with PING / READ / WRITE / REG WRITE / ACTION /
  SYNC WRITE / SYNC READ / RESET instructions.
- **Stock LeRobot drives them as `sts3215`.** Hiwonder's `lerobot.zip` is a clone
  of `huggingface/lerobot` at commit `882c80d` (v0.3.4). In the motor, robot, and
  teleoperator code (the only parts compared), the sole changes are in
  `software/hiwonder-lerobot-882c80d.patch`: sync read/write `num_retry` 0 → 1,
  failed-sync log messages raised from debug to info, and an added `so101.mdx` doc.
  The retry suggests the HX servos occasionally drop sync packets.
  **Caution (2026-11-09):** this is Hiwonder's claim, not something we have verified. Their HX register
  table v3.7 (`docs/datasheets/hiwonder-hx-servos/`) disagrees with the STS3215 map on the P/D/I,
  torque-limit and torque-enable addresses, so check which map the servo firmware uses before any
  register write (`docs/DECISIONS.md` P5). We don't use LeRobot (H5).
- **Print files are Hiwonder's own.** `print/*.3mf` (Bambu Studio projects, 2 plates each) and
  `cad/*.3dm` (Rhino) are Hiwonder's arm geometry. However, the HX servos share the STS3215's outer
  body (45.2 × 24.7 × 35 mm per both datasheets; 45.23 × 24.73 measured from the HX STEP), so the
  upstream `../SO-ARM100/STL` parts *may* fit. Horn spline and hole positions are unverified;
  test with `../SO-ARM100/STL/Gauges/Gauge_0.STL`. Servo docs: `docs/datasheets/hiwonder-hx-servos/`.
- **No HX-specific simulation model.** `urdf_model_download_link.txt` just points to
  upstream `TheRobotStudio/SO-ARM100/Simulation/SO101` (STS3215 geometry and motor params).

## Contents

| File | What it is | Original name in Drive |
|---|---|---|
| `manuals/SO-ARM101_User_Manual.pdf` | 60-page user manual: assembly, wiring, servo IDs, calibration, teleop, data collection, training, inference, dual-arm | `1. Tutorials/1. Hiwonder SO-ARM101 6-Axis Robotic Arm User Manual.pdf` |
| `manuals/HX-30HM_Communication_Protocol_v1.0.pdf` | 9-page bus servo protocol (v1.0, 2025-09-25). References an "HX-35HM memory table" that isn't in the folder | `1. Tutorials/Appendix/BusLinker_v3.0.pdf` (misnamed upstream) |
| `manuals/Magnetic_Encoder_Servo_Firmware_Flashing_Tutorial.pdf` | 7-page servo firmware flashing guide | `1. Tutorials/2. Magnetic Encoder Servo Firmware Flashing Tutorial.pdf` |
| `manuals/Appendix_Magnetic_Encoder_Servo_Firmware_Flashing_Tutorial.pdf` | 6-page variant of the above (different file) | `1. Tutorials/Appendix/Magnetic Encoder Servo Firmware Flashing Tutorial.pdf` |
| `print/Black_LeRobot.3mf`, `print/White_LeRobot.3mf` | Bambu Studio print projects (follower = black, leader = white) | `3. Technical Specifications & Drawings/` |
| `cad/follower_arm_black.3dm`, `cad/leader_arm_white.3dm` | Rhino 3D CAD of both arms (~160 MB each) | `3. Technical Specifications & Drawings/` |
| `software/hiwonder-lerobot-882c80d.patch` | Diff of Hiwonder's `lerobot.zip` vs upstream `huggingface/lerobot@882c80d` (motors/robots/teleoperators) | derived from `2. Software Tools & Source Code/Source Code/lerobot.zip` |
| `Video_Links.txt` | YouTube links: assembly, camera, wiring, servo IDs, calibration, teleop, HX-30HM bracket assembly | `1. Tutorials/Video Tutorials/Video Links.txt` |
| `urdf_model_download_link.txt` | Points to upstream SO101 simulation | `3. Technical Specifications & Drawings/` |

## Not downloaded (fetch from the Drive folder if needed)

| Drive path | Why skipped |
|---|---|
| `1. Tutorials/Appendix/HX-10HM Servo Firmware/*.sfw` (V3.11) | Binary firmware; fetch when actually reflashing |
| `1. Tutorials/Appendix/HX-30HM {New,Old,V3.15} Version Servo Firmware/*.sfw` (V3.11, V3.14, V3.15) | Binary firmware |
| `1. Tutorials/Appendix/ServoStudio_v0.1.5.exe` | Windows servo config tool (executable) |
| `2. Software Tools & Source Code/Source Code/lerobot.zip` (200 MB) | Upstream LeRobot clone; only the patch above differs |
| `2. Software Tools & Source Code/Software Tools/` (Miniconda, MobaXterm installers) | Generic third-party installers |
