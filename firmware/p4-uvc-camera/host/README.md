# host/: the camera firmware built for a PC

`main/uvc_camera_main.c` compiled **unchanged** into a shared library, with stand-ins for everything under
it, so the firmware's logic can run (and be tested) inside the simulation before it is flashed.

```
picture from the simulation
   │  (the host program answers FW_EV_NEED_FRAME)
   ▼
video_fake.c   sensor: one frame every 1/28 s into a queued capture buffer, flips, exposure
   │           capture device and JPEG encoder as the firmware drives them (open / ioctl / mmap)
   ▼
main/uvc_camera_main.c   THE FIRMWARE: camera task, start/stop, locking, idle stop, bad-frame recovery
   ▼
uvc_fake.c     usb_device_uvc's frame task (copied line for line) and a USB link that takes time
   │  (FW_EV_FRAME_OUT)
   ▼
the PC side    sim/so101sim/camera_fw.py: V4L2 buffer queue, JPEG decode
```

| File | What |
|---|---|
| `host_api.c` | Includes the firmware source and exports the C interface the simulation loads (`fw_boot`, `fw_run`, `fw_host_commit`, `fw_set`, `fw_get`, log and misuse lists) |
| `rtos.c` | FreeRTOS stand-in: tasks, mutex, binary semaphore, queue, task notifications, tick count, on **virtual time** |
| `video_fake.c` | esp_video stand-in: `/dev/video0` (sensor + ISP) and `/dev/video10` (JPEG encoder), IMX708 mode table, `esp_video_init`, AGC switch, heap |
| `uvc_fake.c` | usb_device_uvc stand-in: the PC's actions (select a size, close the stream, suspend) and frame transfer time |
| `include/` | Stand-in headers with the names the firmware includes; `host_shim.h` redirects its `open`/`ioctl`/`mmap` calls |

It is built and loaded by `sim/so101sim/camera_fw.py` (cached in `sim/.cache/p4cam/`). By hand:

```bash
cc -shared -fPIC -DNDEBUG -Ihost/include -include host_shim.h \
   host/host_api.c host/rtos.c host/video_fake.c host/uvc_fake.c -o p4cam.so -lpthread
```

Image options are passed like the real build's Kconfig symbols: `-DCONFIG_P4CAM_HFLIP=1`,
`-DCONFIG_P4CAM_FIXED_EXPOSURE=1 -DCONFIG_P4CAM_EXPOSURE_US=5000`, `-DCONFIG_P4CAM_JPEG_QUALITY=60`, and so on.

## How time works

Every firmware task is a thread, but only one runs at a time, and only when the scheduler says so; a run is
exactly repeatable. Code takes no time. Time moves when every task is blocked, and jumps to the next
timeout or device event. The tick is 10 ms, as in `sdkconfig` (`CONFIG_FREERTOS_HZ=100`), so `vTaskDelay(1)`
and the UVC task's frame pacing land on 10 ms boundaries, as on the board.

The host program calls `fw_run(until)`. It returns when virtual time reaches `until`, or earlier with an
event: the sensor wants a picture, the encoder wants a JPEG, or a frame has arrived on the PC.

## The stand-ins check the firmware

Each device reports misuse, and the tests require that no run produces any:

- an ioctl on a closed device, or in the wrong state (dequeue while not streaming, and so on);
- a buffer queued twice; a sensor mode or format changed while buffers are allocated;
- a device closed, or buffers released, with buffers still mapped; `esp_video_deinit` with a device open;
- an encoder input that is a capture buffer already handed back to the sensor, or not 128-byte aligned;
- a frame handed to USB that is not in a live, mapped buffer (use after teardown), or has zero length;
- a mutex taken twice by one task, or given by a task that does not hold it.

`sim/tests/test_camera_fw.py` drives the capture stand-in the wrong way on purpose to prove that these
reports fire.

## What is modelled, and from what

| Behaviour | Source |
|---|---|
| Sizes 1920×1080, 1280×720, 1024×768, 800×600, 640×480, all 28 fps | The vendored IMX708 driver's mode table |
| Setting the sensor mode fixes the Bayer phase (a flip set later swaps red and blue) and resets exposure and gain | The firmware's comments, CAMERA-TECHNICAL-REFERENCE |
| With auto exposure on, a written exposure does not hold; out-of-range exposure is rejected | Same |
| A misaligned encoder input gives a corrupt JPEG; a failed encode completes with 0 bytes | Same |
| `esp_video_init` succeeds with no sensor; opening the capture device then fails | Same |
| UVC task: 30 fps pacing on the tick, one frame in flight, no "stream closed" callback, `start_cb` on the TinyUSB task | `usb_device_uvc` 1.2.0~1 source |
| A zero-length frame is refused and the UVC task then waits forever | README of this firmware |

**Placeholders, not measured** (`fw_set` names; set them from `UvcCamera(..., name=value)`):

| Setting | Default | Meaning |
|---|---:|---|
| `init_us` | 400 000 | `esp_video_init`: sensor detect, ISP and JPEG bring-up |
| `sensor_mode_us` | 100 000 | Setting the sensor mode |
| `deinit_us` | 50 000 | `esp_video_deinit` |
| `readout_us` | 20 000 | Exposure until the frame is in a capture buffer |
| `jpeg_us_per_mp` | 8 000 | Encode time per megapixel |
| `usb_bytes_per_s` | 8 184 000 | Isochronous link: 1023 bytes per 125 µs |

Faults to inject: `sensor_present`, `sensor_stalled`, `init_fail`, `misalign`, `jpeg_fail`, `jpeg_queue_error`.

## Limits

- The stand-in headers were written by hand against esp_video 2.5.0 and usb_device_uvc 1.2.0~1. They are not
  generated from those components, so an upgrade needs a second look here.
- The scheduler never interrupts a task in the middle of plain code. A race that needs an interrupt between
  two statements will not show up. Stack sizes are not modelled (the 4 KB TinyUSB stack is why the camera
  task exists).
- The sensor driver, the patched esp_video files, the ISP, auto exposure / white balance / focus, the real
  JPEG encoder and the USB descriptors are not run. Pictures are the simulation's renders: sharpness,
  noise, colour and JPEG size are not those of the real camera, so **frame rates over USB for large sizes
  are optimistic** (simulated frames compress far better than real ones).
