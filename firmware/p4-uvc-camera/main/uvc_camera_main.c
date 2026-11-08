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
 *  - Each UVC start selects the IMX708 mode matching the requested size with
 *    VIDIOC_S_SENSOR_FMT, and each stop tears the whole video stack down. The
 *    driver's modes are different crops, and changing geometry under allocated
 *    buffers is unsafe (CAMERA-TECHNICAL-REFERENCE section 5).
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
#include "esp_err.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "esp_video_ioctl.h"
#include "linux/videodev2.h"
#include "imx708.h"
#include "usb_device_uvc.h"
#include "uvc_frame_config.h"

#define CAP_BUFFER_COUNT   2
#define JPEG_INPUT_ALIGN   128
#define JPEG_QUALITY       80

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
} cam_t;

/* USB string descriptors live in usb_device_uvc (tusb/usb_descriptors.c);
 * index 3 is the serial number. Not declared in a public header. */
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
        esp_video_deinit();
    }
    heap_caps_free(c->stage);

    memset(c, 0, sizeof(*c));
    c->cap_fd = c->jpeg_fd = -1;
}

static esp_err_t set_jpeg_quality(int fd, int quality)
{
    struct v4l2_ext_control ctrl = {.id = V4L2_CID_JPEG_COMPRESSION_QUALITY, .value = quality};
    struct v4l2_ext_controls ctrls = {.ctrl_class = V4L2_CID_JPEG_CLASS, .count = 1, .controls = &ctrl};
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0 ? ESP_OK : ESP_FAIL;
}

#define CHECK_IOCTL(fd, req, arg) do { \
        if (ioctl((fd), (req), (arg)) != 0) { \
            ESP_LOGE(TAG, "%s failed (line %d)", #req, __LINE__); \
            goto fail; \
        } \
    } while (0)

static esp_err_t video_start_cb(uvc_format_t uvc_format, int width, int height, int rate, void *ctx)
{
    cam_t *c = (cam_t *)ctx;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    struct v4l2_buffer buf;
    int type;

    ESP_LOGI(TAG, "UVC start: %dx%d @ %d fps requested", width, height, rate);
    if (uvc_format != UVC_FORMAT_JPEG) {
        ESP_LOGE(TAG, "only MJPEG is supported");
        return ESP_ERR_NOT_SUPPORTED;
    }

    const esp_cam_sensor_format_t *mode = imx708_format_by_size(width, height);
    if (!mode) {
        ESP_LOGE(TAG, "no IMX708 mode for %dx%d", width, height);
        return ESP_ERR_NOT_SUPPORTED;
    }

    cam_teardown(c);  /* defensive: a stop may not have been delivered */
    if (esp_video_init(&s_video_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed (is the camera connected?)");
        return ESP_FAIL;
    }
    c->video_inited = true;

    c->cap_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDWR);
    c->jpeg_fd = open(ESP_VIDEO_JPEG_DEVICE_NAME, O_RDWR);
    if (c->cap_fd < 0 || c->jpeg_fd < 0) {
        ESP_LOGE(TAG, "failed to open video devices");
        goto fail;
    }

    /* Sensor mode first, then the capture format, then buffers. */
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
    if (set_jpeg_quality(c->jpeg_fd, JPEG_QUALITY) != ESP_OK) {
        ESP_LOGW(TAG, "failed to set JPEG quality");
    }

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

    ESP_LOGI(TAG, "streaming %dx%d MJPEG (IMX708 mode %s)", width, height, mode->name ? mode->name : "?");
    return ESP_OK;

fail:
    cam_teardown(c);
    return ESP_FAIL;
}

static void video_stop_cb(void *ctx)
{
    ESP_LOGI(TAG, "UVC stop");
    cam_teardown((cam_t *)ctx);
}

static uvc_fb_t *video_fb_get_cb(void *ctx)
{
    cam_t *c = (cam_t *)ctx;
    struct v4l2_buffer cap = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    struct v4l2_buffer in = {.index = 0, .type = V4L2_BUF_TYPE_VIDEO_OUTPUT, .memory = V4L2_MEMORY_USERPTR};
    struct v4l2_buffer out = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    struct v4l2_format fmt = {.type = V4L2_BUF_TYPE_VIDEO_CAPTURE};

    if (c->cap_fd < 0 || ioctl(c->cap_fd, VIDIOC_DQBUF, &cap) != 0) {
        return NULL;
    }

    uint8_t *src = c->cap_buf[cap.index];
    if (c->stage) {
        memcpy(c->stage, src, cap.bytesused);
        src = c->stage;
    }
    in.m.userptr = (unsigned long)src;
    in.length = cap.bytesused;

    bool ok = ioctl(c->jpeg_fd, VIDIOC_QBUF, &in) == 0 &&
              ioctl(c->jpeg_fd, VIDIOC_DQBUF, &out) == 0;
    /* Give the capture buffer back as soon as the encoder is done with it. */
    ioctl(c->cap_fd, VIDIOC_QBUF, &cap);
    ok = ok && ioctl(c->jpeg_fd, VIDIOC_DQBUF, &in) == 0 &&
         ioctl(c->jpeg_fd, VIDIOC_G_FMT, &fmt) == 0;
    if (!ok) {
        ESP_LOGE(TAG, "JPEG encode failed");
        return NULL;
    }

    int64_t us = esp_timer_get_time();
    c->fb.buf = c->jpeg_buf;
    c->fb.len = out.bytesused;
    c->fb.width = fmt.fmt.pix.width;
    c->fb.height = fmt.fmt.pix.height;
    c->fb.format = UVC_FORMAT_JPEG;
    c->fb.timestamp.tv_sec = us / 1000000;
    c->fb.timestamp.tv_usec = us % 1000000;
    return &c->fb;
}

static void video_fb_return_cb(uvc_fb_t *fb, void *ctx)
{
    cam_t *c = (cam_t *)ctx;
    struct v4l2_buffer out = {.index = 0, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP};
    if (c->jpeg_fd >= 0) {
        ioctl(c->jpeg_fd, VIDIOC_QBUF, &out);
    }
}

void app_main(void)
{
    static cam_t cam;
    memset(&cam, 0, sizeof(cam));
    cam.cap_fd = cam.jpeg_fd = -1;

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
        .cb_ctx = &cam,
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
