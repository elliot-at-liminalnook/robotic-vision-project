/*
 * ESP32-P4 + Raspberry Pi Camera Module 3 (IMX708) as a USB UVC (MJPEG) webcam.
 *
 * Pipeline: IMX708 --MIPI CSI RAW10--> P4 ISP --RGB565--> HW JPEG --> USB UVC.
 * USB is the P4's native High-Speed port (the 4-pin MX1.25 "USB" header on the
 * Waveshare ESP32-P4-WIFI6 board), not the USB-C port (CH343 serial bridge,
 * which stays the log console).
 *
 * Derived from Espressif's esp_video 2.5.0 "uvc" example (ESPRESSIF MIT).
 * Differences from that example:
 *  - Explicit esp_video_init() for this board: SCCB on I2C0 SDA=GPIO7 SCL=GPIO8,
 *    IMX708 sensor + DW9807 lens motor (values from esp32-stereo-camera Camera Lab).
 *  - Each UVC commit selects the IMX708 mode matching the requested size with
 *    VIDIOC_S_SENSOR_FMT, and each stop tears the whole video stack down. The
 *    driver's modes are different crops, and changing geometry under allocated
 *    buffers is unsafe (CAMERA-TECHNICAL-REFERENCE section 5).
 *  - A dedicated camera task owns bring-up and teardown (the TinyUSB task that
 *    delivers the UVC callbacks only has a 4 KB stack). It also stops the
 *    pipeline when the host stops asking for frames, because usb_device_uvc
 *    has no "stream closed" callback, and restarts it when frames are wanted.
 *  - One mutex serializes frame capture against bring-up/teardown.
 *  - Capture buffers that aren't 128-byte aligned are copied into an aligned
 *    staging buffer before JPEG, because misaligned input produced corrupt JPEGs
 *    on this board (CAMERA-TECHNICAL-REFERENCE section 19).
 *  - USB serial number = P4 base MAC, so two boards get distinct, stable
 *    /dev/v4l/by-id names on Linux from one firmware image.
 */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "esp_video_isp_pipeline.h"
#include "linux/videodev2.h"
#include "imx708.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

#define CAP_BUFFER_COUNT   2
#define JPEG_INPUT_ALIGN   128

#define CAM_TASK_STACK     8192   /* esp_video_init + sensor detect + ISP init need ~3 KB */
#define CAM_TASK_PRIORITY  5
#define CAM_POLL_MS        100
#define DQBUF_TIMEOUT_MS   200    /* bounds how long a frame grab can hold s_lock */
#define IDLE_STOP_MS       1500   /* no frame requested for this long -> stop the pipeline */
#define RESTART_BACKOFF_MS 1000
#define BAD_FRAME_LIMIT    30     /* consecutive failed frames before a full restart */

static const char *TAG = "p4_uvc";

/* ---- Board camera wiring (Waveshare ESP32-P4-WIFI6, 22-pin CSI) ---------- */
static const esp_video_init_csi_config_t s_csi_cfg[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = {.port = 0, .scl_pin = 8, .sda_pin = 7},
        .freq = 100000,
    },
    .reset_pin = -1,
    .pwdn_pin = -1,
}};
static const esp_video_init_cam_motor_config_t s_motor_cfg[] = {{
    .sccb_config = {
        .init_sccb = true,
        .i2c_config = {.port = 0, .scl_pin = 8, .sda_pin = 7},
        .freq = 100000,
    },
    .reset_pin = -1,
    .pwdn_pin = -1,
    .signal_pin = -1,
}};
static const esp_video_init_config_t s_video_cfg = {
    .csi = s_csi_cfg,
    .cam_motor = s_motor_cfg,
};

/* ---- Stream state -------------------------------------------------------- */
typedef struct {
    bool video_inited;
    int cap_fd;                              /* MIPI CSI capture device */
    int jpeg_fd;                             /* HW JPEG m2m device */
    uint8_t *cap_buf[CAP_BUFFER_COUNT];
    size_t cap_len[CAP_BUFFER_COUNT];
    uint8_t *stage;                          /* aligned copy, only if needed */
    size_t stage_len;
    uint8_t *jpeg_buf;
    size_t jpeg_len;
    uvc_fb_t fb;
    bool running;                            /* pipeline is up and streaming */
    int bad_frames;                          /* consecutive failed frame grabs */
} cam_t;

/* A start/stop request from the UVC callbacks to the camera task. */
typedef struct {
    bool start;                              /* false = stop */
    int width;
    int height;
    esp_err_t result;
} cam_req_t;

static cam_t s_cam = {.cap_fd = -1, .jpeg_fd = -1};
/* Guards s_cam and the video devices. Held by the camera task during
 * bring-up/teardown, and by the UVC task from a successful fb_get until the
 * matching fb_return (the frame it hands out lives in the mmap'd JPEG buffer,
 * which teardown unmaps). */
static SemaphoreHandle_t s_lock;
static TickType_t s_last_fb_get;             /* last time the host wanted a frame; under s_lock */
static QueueHandle_t s_req_q;                /* cam_req_t *, to the camera task */
static SemaphoreHandle_t s_req_done;

/* USB string descriptors live in usb_device_uvc (tusb/usb_descriptors.c);
 * index 3 is the serial number. Not declared in a public header, which is why
 * main/idf_component.yml pins that component to an exact version. */
extern char const *string_desc_arr[];
static char s_serial[13];

static void cam_teardown(cam_t *c)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (c->cap_fd >= 0) {
        ioctl(c->cap_fd, VIDIOC_STREAMOFF, &type);
        for (int i = 0; i < CAP_BUFFER_COUNT; i++) {
            if (c->cap_buf[i]) {
                munmap(c->cap_buf[i], c->cap_len[i]);
            }
        }
        struct v4l2_requestbuffers rel = {.count = 0, .type = type, .memory = V4L2_MEMORY_MMAP};
        ioctl(c->cap_fd, VIDIOC_REQBUFS, &rel);
        close(c->cap_fd);
    }
    if (c->jpeg_fd >= 0) {
        type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
        ioctl(c->jpeg_fd, VIDIOC_STREAMOFF, &type);
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(c->jpeg_fd, VIDIOC_STREAMOFF, &type);
        if (c->jpeg_buf) {
            munmap(c->jpeg_buf, c->jpeg_len);
        }
        close(c->jpeg_fd);
    }
    if (c->video_inited) {
        esp_err_t err = esp_video_deinit();
        if (err != ESP_OK) {
            /* Not recoverable here; the next esp_video_init() will likely fail too. */
            ESP_LOGE(TAG, "esp_video_deinit failed: %s", esp_err_to_name(err));
        }
    }
    heap_caps_free(c->stage);

    memset(c, 0, sizeof(*c));
    c->cap_fd = c->jpeg_fd = -1;
}

static bool set_ctrl(int fd, uint32_t ctrl_class, uint32_t id, int32_t value, const char *name)
{
    struct v4l2_ext_control ctrl = {.id = id, .value = value};
    struct v4l2_ext_controls ctrls = {.ctrl_class = ctrl_class, .count = 1, .controls = &ctrl};
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
        ESP_LOGE(TAG, "failed to set %s = %d", name, (int)value);
        return false;
    }
    ESP_LOGI(TAG, "%s = %d", name, (int)value);
    return true;
}

/* Kconfig "P4 UVC camera" options that must be in place before the sensor mode
 * is set: esp_video latches the Bayer phase (which a flip rotates) in
 * VIDIOC_S_SENSOR_FMT, so a flip set later gives wrong colours. */
static void apply_pre_format_settings(int cap_fd)
{
    /* The sensor comes out of esp_video_init() unflipped, so only "on" needs setting. */
#ifdef CONFIG_P4CAM_HFLIP
    set_ctrl(cap_fd, V4L2_CID_USER_CLASS, V4L2_CID_HFLIP, 1, "hflip");
#endif
#ifdef CONFIG_P4CAM_VFLIP
    set_ctrl(cap_fd, V4L2_CID_USER_CLASS, V4L2_CID_VFLIP, 1, "vflip");
#endif
#ifdef CONFIG_P4CAM_FIXED_EXPOSURE
    /* Stop the IPA's auto exposure/gain loop, or it overwrites the fixed
     * values on the next frame. AWB and the other ISP algorithms keep running. */
    esp_err_t err = esp_video_isp_pipeline_set_agc_status(ESP_VIDEO_ISP_PIPELINE_AGC_DISABLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "could not disable auto exposure (%s); fixed exposure will not hold", esp_err_to_name(err));
    }
#endif
}

/* Options applied once the stream is running (setting the sensor mode resets
 * exposure and gain, so these have to come after it). */
static void apply_post_start_settings(int cap_fd)
{
#ifdef CONFIG_P4CAM_FIXED_EXPOSURE
    /* The valid exposure range depends on the mode's frame length; clamp to it
     * (esp_video rejects out-of-range values outright). Unit: microseconds. */
    struct v4l2_query_ext_ctrl q = {.id = V4L2_CID_EXPOSURE_ABSOLUTE};
    int32_t us = CONFIG_P4CAM_EXPOSURE_US;
    if (ioctl(cap_fd, VIDIOC_QUERY_EXT_CTRL, &q) == 0 && (us < q.minimum || us > q.maximum)) {
        int32_t clamped = us < q.minimum ? (int32_t)q.minimum : (int32_t)q.maximum;
        ESP_LOGW(TAG, "exposure %d us outside this mode's range; using %d us", (int)us, (int)clamped);
        us = clamped;
    }
    set_ctrl(cap_fd, V4L2_CID_CAMERA_CLASS, V4L2_CID_EXPOSURE_ABSOLUTE, us, "exposure_us");
    set_ctrl(cap_fd, V4L2_CID_USER_CLASS, V4L2_CID_GAIN, CONFIG_P4CAM_GAIN_INDEX, "gain_index");
#endif
#ifdef CONFIG_P4CAM_FIXED_FOCUS
    set_ctrl(cap_fd, V4L2_CID_CAMERA_CLASS, V4L2_CID_FOCUS_ABSOLUTE, CONFIG_P4CAM_FOCUS_POSITION, "focus_position");
#endif
}

#define CHECK_IOCTL(fd, req, arg) do { \
        if (ioctl((fd), (req), (arg)) != 0) { \
            ESP_LOGE(TAG, "%s failed (line %d)", #req, __LINE__); \
            goto fail; \
        } \
    } while (0)

/* Bring the whole pipeline up for one frame size. Camera task only, s_lock held. */
static esp_err_t cam_start(cam_t *c, int width, int height)
{
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    struct timeval dq_timeout = {.tv_sec = 0, .tv_usec = DQBUF_TIMEOUT_MS * 1000};
    int type;

    cam_teardown(c);  /* the previous mode may still be up */

    const esp_cam_sensor_format_t *mode = imx708_format_by_size(width, height);
    if (!mode) {
        ESP_LOGE(TAG, "no IMX708 mode for %dx%d", width, height);
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t err = esp_video_init(&s_video_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(err));
        return ESP_FAIL;
    }
    c->video_inited = true;

    c->cap_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    c->jpeg_fd = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);
    if (c->cap_fd < 0 || c->jpeg_fd < 0) {
        /* esp_video_init() returns OK when no sensor answers; the missing
         * capture device is the first sign of it. */
        ESP_LOGE(TAG, "failed to open video devices (capture %s, JPEG %s). "
                 "Is the camera connected and the ribbon cable fully seated?",
                 c->cap_fd < 0 ? "missing" : "ok", c->jpeg_fd < 0 ? "missing" : "ok");
        goto fail;
    }

    /* Without a timeout DQBUF blocks forever if the sensor stalls, and the
     * frame grab would then hold s_lock forever. */
    CHECK_IOCTL(c->cap_fd, VIDIOC_S_DQBUF_TIMEOUT, &dq_timeout);
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_S_DQBUF_TIMEOUT, &dq_timeout);

    /* Flips, then sensor mode, then the capture format, then buffers. */
    apply_pre_format_settings(c->cap_fd);
    CHECK_IOCTL(c->cap_fd, VIDIOC_S_SENSOR_FMT, (void *)mode);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    CHECK_IOCTL(c->cap_fd, VIDIOC_S_FMT, &fmt);

    memset(&req, 0, sizeof(req));
    req.count = CAP_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    CHECK_IOCTL(c->cap_fd, VIDIOC_REQBUFS, &req);

    bool need_stage = false;
    for (int i = 0; i < CAP_BUFFER_COUNT; i++) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        CHECK_IOCTL(c->cap_fd, VIDIOC_QUERYBUF, &buf);
        c->cap_buf[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->cap_fd, buf.m.offset);
        if (c->cap_buf[i] == MAP_FAILED) {
            c->cap_buf[i] = NULL;
            ESP_LOGE(TAG, "mmap capture buffer %d failed", i);
            goto fail;
        }
        c->cap_len[i] = buf.length;
        if ((uintptr_t)c->cap_buf[i] % JPEG_INPUT_ALIGN) {
            need_stage = true;
        }
        CHECK_IOCTL(c->cap_fd, VIDIOC_QBUF, &buf);
    }

    /* Note: VIDIOC_G_FMT's sizeimage is 0 in this esp_video build; size from geometry. */
    const size_t frame_bytes = (size_t)width * height * 2;
    if (need_stage) {
        c->stage_len = (frame_bytes + JPEG_INPUT_ALIGN - 1) / JPEG_INPUT_ALIGN * JPEG_INPUT_ALIGN;
        c->stage = heap_caps_aligned_alloc(JPEG_INPUT_ALIGN, c->stage_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!c->stage) {
            ESP_LOGE(TAG, "staging alloc (%u bytes) failed", (unsigned)c->stage_len);
            goto fail;
        }
        ESP_LOGW(TAG, "capture buffers not %d-byte aligned; using aligned staging copy", JPEG_INPUT_ALIGN);
    }

    /* JPEG encoder: OUTPUT queue takes RGB565 (USERPTR), CAPTURE queue yields JPEG. */
    set_ctrl(c->jpeg_fd, V4L2_CID_JPEG_CLASS, V4L2_CID_JPEG_COMPRESSION_QUALITY,
             CONFIG_P4CAM_JPEG_QUALITY, "jpeg_quality");

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_S_FMT, &fmt);

    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    req.memory = V4L2_MEMORY_USERPTR;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_REQBUFS, &req);

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_S_FMT, &fmt);

    memset(&req, 0, sizeof(req));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_REQBUFS, &req);

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_QUERYBUF, &buf);
    c->jpeg_buf = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, c->jpeg_fd, buf.m.offset);
    if (c->jpeg_buf == MAP_FAILED) {
        c->jpeg_buf = NULL;
        ESP_LOGE(TAG, "mmap JPEG buffer failed");
        goto fail;
    }
    c->jpeg_len = buf.length;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_QBUF, &buf);

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_STREAMON, &type);
    type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
    CHECK_IOCTL(c->jpeg_fd, VIDIOC_STREAMON, &type);
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    CHECK_IOCTL(c->cap_fd, VIDIOC_STREAMON, &type);

    apply_post_start_settings(c->cap_fd);

    c->running = true;
    ESP_LOGI(TAG, "streaming %dx%d MJPEG (IMX708 mode %s)", width, height, mode->name ? mode->name : "?");
    return ESP_OK;

fail:
    cam_teardown(c);
    return ESP_FAIL;
}

/* ---- Camera task: owns bring-up and teardown --------------------------- */
static void cam_task(void *arg)
{
    cam_t *c = (cam_t *)arg;
    int width = 0, height = 0;               /* last committed mode; 0 = none yet */
    TickType_t last_start = 0;
    cam_req_t *req;

    for (;;) {
        if (xQueueReceive(s_req_q, &req, pdMS_TO_TICKS(CAM_POLL_MS)) == pdTRUE) {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            if (req->start) {
                width = req->width;
                height = req->height;
                req->result = cam_start(c, width, height);
                last_start = xTaskGetTickCount();
            } else {
                cam_teardown(c);
                req->result = ESP_OK;
            }
            xSemaphoreGive(s_lock);
            xSemaphoreGive(s_req_done);
            continue;
        }

        /* Housekeeping. usb_device_uvc only reports "stop" on USB suspend, not
         * when the host closes the stream, so go by whether frames are being
         * requested: fb_get stamps s_last_fb_get on every call. */
        xSemaphoreTake(s_lock, portMAX_DELAY);
        TickType_t now = xTaskGetTickCount();
        bool wanted = (now - s_last_fb_get) < pdMS_TO_TICKS(IDLE_STOP_MS);
        TickType_t since_start = now - last_start;

        if (c->running && !wanted && since_start >= pdMS_TO_TICKS(IDLE_STOP_MS)) {
            /* (since_start: a fresh commit gets the same grace to begin streaming.) */
            ESP_LOGI(TAG, "host stopped reading frames; camera off");
            cam_teardown(c);
        } else if (wanted && width && since_start >= pdMS_TO_TICKS(RESTART_BACKOFF_MS) &&
                   (!c->running || c->bad_frames >= BAD_FRAME_LIMIT)) {
            /* Frames wanted but the pipeline was idle-stopped, failed to start,
             * or has stopped producing good frames: bring it up again. */
            ESP_LOGW(TAG, "restarting camera at %dx%d (%s)", width, height,
                     c->running ? "too many bad frames" : "was stopped");
            cam_start(c, width, height);
            last_start = xTaskGetTickCount();
        }
        xSemaphoreGive(s_lock);
    }
}

/* Hand a request to the camera task and wait for it to finish. */
static esp_err_t cam_request(bool start, int width, int height)
{
    cam_req_t req = {.start = start, .width = width, .height = height, .result = ESP_FAIL};
    cam_req_t *p = &req;

    xQueueSend(s_req_q, &p, portMAX_DELAY);
    xSemaphoreTake(s_req_done, portMAX_DELAY);
    return req.result;
}

/* ---- usb_device_uvc callbacks ---------------------------------------------
 * start/stop arrive on the TinyUSB task (start on every UVC commit, stop on USB
 * suspend); fb_get/fb_return on the UVC task. */
static esp_err_t video_start_cb(uvc_format_t uvc_format, int width, int height, int rate, void *ctx)
{
    ESP_LOGI(TAG, "UVC start: %dx%d @ %d fps requested", width, height, rate);
    if (uvc_format != UVC_FORMAT_JPEG) {
        ESP_LOGE(TAG, "only MJPEG is supported");
        return ESP_ERR_NOT_SUPPORTED;
    }
    return cam_request(true, width, height);
}

static void video_stop_cb(void *ctx)
{
    ESP_LOGI(TAG, "UVC stop");
    cam_request(false, 0, 0);
}

/* Capture one frame and JPEG-encode it into c->jpeg_buf. s_lock held.
 * On failure every buffer is back where the next attempt expects it. */
static esp_err_t grab_frame(cam_t *c)
{
    struct v4l2_buffer cap = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    struct v4l2_buffer in = {.index = 0, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_USERPTR};
    struct v4l2_buffer out = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};

    if (ioctl(c->cap_fd, VIDIOC_DQBUF, &cap) != 0) {
        ESP_LOGW(TAG, "no frame from the sensor within %d ms", DQBUF_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    if (cap.bytesused == 0 || (cap.flags & V4L2_BUF_FLAG_ERROR) || (c->stage && cap.bytesused > c->stage_len)) {
        ESP_LOGW(TAG, "bad capture buffer (%u bytes)", (unsigned)cap.bytesused);
        ioctl(c->cap_fd, VIDIOC_QBUF, &cap);
        return ESP_FAIL;
    }

    uint8_t *src = c->cap_buf[cap.index];
    if (c->stage) {
        memcpy(c->stage, src, cap.bytesused);
        src = c->stage;
    }
    in.m.userptr = (unsigned long)src;
    in.length = cap.bytesused;

    /* The encode runs inside DQBUF(out). When the encoder itself fails,
     * esp_video still completes both buffers, with out.bytesused == 0. */
    bool ok = ioctl(c->jpeg_fd, VIDIOC_QBUF, &in) == 0 &&
              ioctl(c->jpeg_fd, VIDIOC_DQBUF, &out) == 0;
    /* Give the capture buffer back as soon as the encoder is done with it. */
    ioctl(c->cap_fd, VIDIOC_QBUF, &cap);
    if (!ok) {
        /* Buffer state unknown; BAD_FRAME_LIMIT of these restarts the pipeline. */
        ESP_LOGE(TAG, "JPEG encoder queue error");
        return ESP_FAIL;
    }
    ok = ioctl(c->jpeg_fd, VIDIOC_DQBUF, &in) == 0 &&
         ioctl(c->jpeg_fd, VIDIOC_G_FMT, &fmt) == 0 &&
         out.bytesused != 0 && !(out.flags & V4L2_BUF_FLAG_ERROR);
    if (!ok) {
        /* A zero-length frame must not reach usb_device_uvc: TinyUSB refuses
         * it, but the UVC task has already marked itself busy and would wait
         * forever. Re-queue the JPEG buffer (fb_return won't run) and drop. */
        ESP_LOGE(TAG, "JPEG encode failed");
        ioctl(c->jpeg_fd, VIDIOC_QBUF, &out);
        return ESP_FAIL;
    }

    int64_t us = esp_timer_get_time();
    c->fb.buf = c->jpeg_buf;
    c->fb.len = out.bytesused;
    c->fb.width = fmt.fmt.pix.width;
    c->fb.height = fmt.fmt.pix.height;
    c->fb.format = UVC_FORMAT_JPEG;
    c->fb.timestamp.tv_sec = us / 1000000;
    c->fb.timestamp.tv_usec = us % 1000000;
    return ESP_OK;
}

static uvc_fb_t *video_fb_get_cb(void *ctx)
{
    cam_t *c = (cam_t *)ctx;

    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_last_fb_get = xTaskGetTickCount();
    if (!c->running) {
        /* Idle-stopped or failed; the camera task sees the fresh timestamp and
         * restarts it. Sleep so the caller doesn't spin while that happens. */
        xSemaphoreGive(s_lock);
        vTaskDelay(pdMS_TO_TICKS(CAM_POLL_MS));
        return NULL;
    }
    if (grab_frame(c) != ESP_OK) {
        c->bad_frames++;
        xSemaphoreGive(s_lock);
        return NULL;
    }
    c->bad_frames = 0;
    return &c->fb;  /* s_lock stays held until video_fb_return_cb */
}

/* usb_device_uvc calls this exactly once after every non-NULL fb_get, from the
 * same task, once it has copied the frame out. */
static void video_fb_return_cb(uvc_fb_t *fb, void *ctx)
{
    cam_t *c = (cam_t *)ctx;
    struct v4l2_buffer out = {.index = 0, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};

    ioctl(c->jpeg_fd, VIDIOC_QBUF, &out);
    xSemaphoreGive(s_lock);
}

void app_main(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_req_done = xSemaphoreCreateBinary();
    s_req_q = xQueueCreate(1, sizeof(cam_req_t *));
    assert(s_lock && s_req_done && s_req_q);
    BaseType_t created = xTaskCreate(cam_task, "camera", CAM_TASK_STACK, &s_cam, CAM_TASK_PRIORITY, NULL);
    assert(created == pdPASS);

    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_BASE));
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    string_desc_arr[3] = s_serial;
    ESP_LOGI(TAG, "USB serial number: %s", s_serial);

    /* The JPEG output buffer must hold the largest advertised frame. */
    uvc_device_config_t config = {
        .start_cb = video_start_cb,
        .fb_get_cb = video_fb_get_cb,
        .fb_return_cb = video_fb_return_cb,
        .stop_cb = video_stop_cb,
        .cb_ctx = &s_cam,
        .uvc_buffer_size = UVC_FRAMES_INFO[0][0].width * UVC_FRAMES_INFO[0][0].height,
    };
    config.uvc_buffer = heap_caps_malloc(config.uvc_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    assert(config.uvc_buffer);

    for (int i = 0; i < UVC_FRAME_NUM; i++) {
        ESP_LOGI(TAG, "advertised frame %d: %dx%d @ %d fps", i,
                 UVC_FRAMES_INFO[0][i].width, UVC_FRAMES_INFO[0][i].height, UVC_FRAMES_INFO[0][i].rate);
    }

    ESP_ERROR_CHECK(uvc_device_config(0, &config));
    ESP_ERROR_CHECK(uvc_device_init());
    ESP_LOGI(TAG, "UVC device ready; waiting for host to open the camera");
}
