/*
 * Stand-ins for the hardware under the firmware: esp_video's MIPI-CSI capture
 * device (sensor + ISP) and its hardware JPEG encoder, as the firmware drives
 * them through open/ioctl/mmap. Pixels come from the host program: the sensor
 * asks it for each frame (FW_EV_NEED_FRAME) and the encoder asks it to
 * compress one (FW_EV_NEED_JPEG).
 *
 * These also check how the firmware uses the devices and report every misuse
 * (host_violation): ioctls in the wrong state, buffers queued twice, a sensor
 * mode changed under allocated buffers, a device closed with buffers still
 * mapped, an encoder input the sensor could overwrite, and so on.
 *
 * Behaviour taken from the firmware's own notes and CAMERA-TECHNICAL-REFERENCE,
 * not from running esp_video:
 *   - setting the sensor mode latches the flips (Bayer phase) and resets exposure and gain;
 *   - exposure outside the mode's range is rejected;
 *   - with auto exposure on, a written exposure is overwritten by the next frame;
 *   - an encoder input that isn't 128-byte aligned gives a corrupt JPEG;
 *   - a failed encode still completes both buffers, with 0 bytes used.
 */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "host_shim.h"
#include "host_internal.h"
#undef open
#undef close
#undef ioctl
#undef mmap
#undef munmap

#define FD_CAP  100
#define FD_JPEG 110
#define NBUF 4
#define ALIGN 128
enum { B_USER, B_QUEUED, B_FILLED };

host_params_t host_params = {
    .init_us = 400000, .deinit_us = 50000, .sensor_mode_us = 100000, .readout_us = 20000, .jpeg_us_per_mp = 8000,
    .usb_bytes_per_s = 8184000, .sensor_present = 1,
};
host_stats_t host_stats;

static const esp_cam_sensor_format_t s_modes[] = {   /* the vendored driver's table: all RAW10, 28 fps */
    {"MIPI_2lane_24Minput_RAW10_1920x1080_binned_28fps", 1920, 1080, 28},
    {"MIPI_2lane_24Minput_RAW10_1280x720_binned_28fps", 1280, 720, 28},
    {"MIPI_2lane_24Minput_RAW10_1024x768_binned_28fps", 1024, 768, 28},
    {"MIPI_2lane_24Minput_RAW10_800x600_binned_28fps", 800, 600, 28},
    {"MIPI_2lane_24Minput_RAW10_640x480_binned_28fps", 640, 480, 28},
};

static struct {
    int open, streaming, w, h, nbuf, filling;
    const esp_cam_sensor_format_t *mode;
    struct { uint8_t *base, *mem; size_t len; int state, mapped; uint64_t seq, qorder; int64_t t, ready; } b[NBUF];
    int64_t dq_timeout_us, period_us;
    int hflip, vflip, latched_hflip, latched_vflip, exposure_set, exposure_us, gain, focus;
    uint64_t seq, qctr;
} C = {.filling = -1};

static struct {
    int open, w, h, in_req, out_req, on_in, on_out, quality, dq_failed;
    uint8_t *out;
    size_t out_len;
    int out_mapped, out_state;
    uint32_t out_used;
    const uint8_t *in_ptr;
    int in_state;                 /* 0 none, 1 queued, 2 encoded */
    int64_t dq_timeout_us;
    uint64_t seq;                 /* the sensor frame in the output buffer */
    int64_t t_exposure_us;
} J;

static int s_inited, s_agc_on = 1;
static uint64_t s_last_seq;       /* the capture frame the firmware dequeued last */
static int64_t s_last_t;
static uint32_t s_jpeg_len;
static uint8_t s_mac[6] = {0x30, 0xED, 0xA0, 0x00, 0x00, 0x01};

/* ---- small ESP-IDF pieces ------------------------------------------------- */
const char *esp_err_to_name(esp_err_t e)
{
    switch (e) {
    case ESP_OK: return "ESP_OK";
    case ESP_FAIL: return "ESP_FAIL";
    case ESP_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
    default: return "ESP_ERR";
    }
}
void host_set_mac(const uint8_t *mac) { memcpy(s_mac, mac, 6); }
esp_err_t esp_read_mac(uint8_t *mac, int type) { (void)type; memcpy(mac, s_mac, 6); return ESP_OK; }
int64_t esp_timer_get_time(void) { return host_now_us(); }

void *heap_caps_aligned_alloc(size_t align, size_t size, uint32_t caps)
{
    void *p = NULL;
    (void)caps;
    if (posix_memalign(&p, align < sizeof(void *) ? sizeof(void *) : align, size ? size : 1) != 0) {
        return NULL;
    }
    host_stats.live_allocs++;
    return p;
}
void *heap_caps_malloc(size_t size, uint32_t caps) { return heap_caps_aligned_alloc(16, size, caps); }
void heap_caps_free(void *p)
{
    if (p) {
        host_stats.live_allocs--;
        free(p);
    }
}

const esp_cam_sensor_format_t *imx708_format_by_size(uint16_t width, uint16_t height)
{
    for (size_t i = 0; i < sizeof(s_modes) / sizeof(s_modes[0]); i++) {
        if (s_modes[i].width == width && s_modes[i].height == height) {
            return &s_modes[i];
        }
    }
    return NULL;
}

/* ---- esp_video ------------------------------------------------------------ */
esp_err_t esp_video_init(const esp_video_init_config_t *config)
{
    (void)config;
    if (s_inited) {
        host_violation("esp_video_init called while already initialised");
        return ESP_FAIL;
    }
    host_sleep_us(host_params.init_us);
    if (host_params.init_fail > 0) {
        host_params.init_fail--;
        return ESP_FAIL;
    }
    s_inited = 1;
    s_agc_on = 1;
    host_stats.video_inited = 1;
    return ESP_OK;          /* also when no sensor answers, like the real one */
}

esp_err_t esp_video_deinit(void)
{
    if (!s_inited) {
        host_violation("esp_video_deinit called while not initialised");
        return ESP_FAIL;
    }
    if (C.open || J.open) {
        host_violation("esp_video_deinit with a video device still open");
    }
    host_sleep_us(host_params.deinit_us);
    s_inited = 0;
    host_stats.video_inited = 0;
    return ESP_OK;
}

esp_err_t esp_video_isp_pipeline_set_agc_status(esp_video_isp_pipeline_agc_status_t status)
{
    if (!s_inited) {
        return ESP_FAIL;
    }
    s_agc_on = status == ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE;
    return ESP_OK;
}

/* ---- the sensor ----------------------------------------------------------- */
static void sensor_tick(void)
{
    if (!C.streaming) {
        return;
    }
    host_timer_set(HOST_TIMER_SENSOR, host_now_us() + C.period_us, sensor_tick);
    if (host_params.sensor_stalled) {
        return;
    }
    host_stats.sensor_frames++;
    C.seq++;
    int pick = -1;
    for (int i = 0; i < C.nbuf; i++) {
        if (C.b[i].state == B_QUEUED && (pick < 0 || C.b[i].qorder < C.b[pick].qorder)) {
            pick = i;
        }
    }
    if (pick < 0) {
        host_stats.sensor_dropped++;      /* the firmware holds every buffer: this frame is lost */
        return;
    }
    C.filling = pick;
    memset(&host_event, 0, sizeof(host_event));
    host_event.kind = FW_EV_NEED_FRAME;
    host_event.width = C.w;
    host_event.height = C.h;
    host_event.hflip = C.hflip;
    host_event.vflip = C.vflip;
    host_event.wrong_colours = C.hflip != C.latched_hflip || C.vflip != C.latched_vflip;
    host_event.exposure_us = (C.exposure_set && !s_agc_on) ? C.exposure_us : 0;
    host_event.gain_index = C.gain;
    host_event.ptr = C.b[pick].mem;
    host_event.len = (uint32_t)C.b[pick].len;
    host_event.seq = C.seq;
    host_event.t_exposure_us = host_now_us();
    host_event_post();
}

void host_frame_filled(void)
{
    if (C.filling >= 0 && C.streaming && C.b[C.filling].state == B_QUEUED) {
        C.b[C.filling].state = B_FILLED;
        C.b[C.filling].seq = C.seq;
        C.b[C.filling].t = host_now_us();
        C.b[C.filling].ready = host_now_us() + host_params.readout_us;   /* the firmware can't have it sooner */
        host_wake(&C);
    }
    C.filling = -1;
}

static void cap_free_buffers(void)
{
    for (int i = 0; i < C.nbuf; i++) {
        free(C.b[i].base);
        memset(&C.b[i], 0, sizeof(C.b[i]));
    }
    C.nbuf = 0;
}

static int cap_ioctl(unsigned long req, void *arg)
{
    switch (req) {
    case VIDIOC_S_DQBUF_TIMEOUT: {
        struct timeval *tv = arg;
        C.dq_timeout_us = (int64_t)tv->tv_sec * 1000000 + tv->tv_usec;
        return 0;
    }
    case VIDIOC_S_EXT_CTRLS: {
        struct v4l2_ext_controls *cs = arg;
        for (uint32_t i = 0; i < cs->count; i++) {
            struct v4l2_ext_control *c = &cs->controls[i];
            switch (c->id) {
            case V4L2_CID_HFLIP: C.hflip = c->value != 0; break;
            case V4L2_CID_VFLIP: C.vflip = c->value != 0; break;
            case V4L2_CID_GAIN:
                if (c->value < 0 || c->value > 46) {
                    return -1;
                }
                C.gain = c->value;
                break;
            case V4L2_CID_EXPOSURE_ABSOLUTE:
                if (!C.mode || c->value < 30 || c->value > 1000000 / C.mode->fps) {
                    return -1;                   /* out of the mode's range: rejected */
                }
                C.exposure_us = c->value;
                C.exposure_set = 1;
                break;
            case V4L2_CID_FOCUS_ABSOLUTE: C.focus = c->value; break;
            default: return -1;
            }
        }
        return 0;
    }
    case VIDIOC_QUERY_EXT_CTRL: {
        struct v4l2_query_ext_ctrl *q = arg;
        if (q->id != V4L2_CID_EXPOSURE_ABSOLUTE || !C.mode) {
            return -1;
        }
        q->minimum = 30;
        q->maximum = 1000000 / C.mode->fps;
        return 0;
    }
    case VIDIOC_S_SENSOR_FMT:
        if (C.nbuf || C.streaming) {
            host_violation("sensor mode changed while capture buffers are allocated");
            return -1;
        }
        host_sleep_us(host_params.sensor_mode_us);
        C.mode = arg;
        C.latched_hflip = C.hflip;               /* the ISP's Bayer phase is fixed here */
        C.latched_vflip = C.vflip;
        C.exposure_set = 0;                      /* a mode change resets exposure and gain */
        C.gain = 0;
        C.w = C.h = 0;
        return 0;
    case VIDIOC_S_FMT: {
        struct v4l2_format *f = arg;
        if (C.nbuf) {
            host_violation("capture format changed while buffers are allocated");
            return -1;
        }
        if (!C.mode || f->fmt.pix.width != C.mode->width || f->fmt.pix.height != C.mode->height ||
            f->fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
            return -1;
        }
        C.w = (int)f->fmt.pix.width;
        C.h = (int)f->fmt.pix.height;
        return 0;
    }
    case VIDIOC_REQBUFS: {
        struct v4l2_requestbuffers *r = arg;
        if (C.streaming) {
            host_violation("capture REQBUFS while streaming");
            return -1;
        }
        for (int i = 0; i < C.nbuf; i++) {
            if (C.b[i].mapped) {
                host_violation("capture buffers released while still mapped");
            }
        }
        cap_free_buffers();
        if (r->count == 0) {
            return 0;
        }
        if (!C.w || r->count > NBUF || r->memory != V4L2_MEMORY_MMAP) {
            return -1;
        }
        for (uint32_t i = 0; i < r->count; i++) {
            C.b[i].len = (size_t)C.w * C.h * 2;
            if (posix_memalign((void **)&C.b[i].base, ALIGN, C.b[i].len + ALIGN) != 0) {
                return -1;
            }
            C.b[i].mem = C.b[i].base + (host_params.misalign ? 8 : 0);
        }
        C.nbuf = (int)r->count;
        return 0;
    }
    case VIDIOC_QUERYBUF: {
        struct v4l2_buffer *b = arg;
        if ((int)b->index >= C.nbuf) {
            return -1;
        }
        b->length = (uint32_t)C.b[b->index].len;
        b->m.offset = b->index;
        return 0;
    }
    case VIDIOC_QBUF: {
        struct v4l2_buffer *b = arg;
        if ((int)b->index >= C.nbuf) {
            host_violation("capture QBUF of buffer %u, which does not exist", b->index);
            return -1;
        }
        if (C.b[b->index].state != B_USER) {
            host_violation("capture buffer %u queued twice", b->index);
            return -1;
        }
        C.b[b->index].state = B_QUEUED;
        C.b[b->index].qorder = ++C.qctr;
        return 0;
    }
    case VIDIOC_DQBUF: {
        struct v4l2_buffer *b = arg;
        if (!C.streaming) {
            host_violation("capture DQBUF while not streaming");
            return -1;
        }
        for (;;) {
            int pick = -1;
            for (int i = 0; i < C.nbuf; i++) {
                if (C.b[i].state == B_FILLED && (pick < 0 || C.b[i].seq < C.b[pick].seq)) {
                    pick = i;
                }
            }
            if (pick >= 0 && C.b[pick].ready > host_now_us()) {      /* still being read out */
                int64_t wait = C.b[pick].ready - host_now_us();
                if (C.dq_timeout_us && wait > C.dq_timeout_us) {
                    host_sleep_us(C.dq_timeout_us);
                    return -1;
                }
                host_sleep_us(wait);
                if (!C.streaming) {
                    return -1;
                }
                continue;
            }
            if (pick >= 0) {
                C.b[pick].state = B_USER;
                b->index = (uint32_t)pick;
                b->bytesused = (uint32_t)C.b[pick].len;
                b->flags = 0;
                s_last_seq = C.b[pick].seq;
                s_last_t = C.b[pick].t;
                return 0;
            }
            if (!host_block(&C, C.dq_timeout_us ? C.dq_timeout_us : -1) || !C.streaming) {
                return -1;
            }
        }
    }
    case VIDIOC_STREAMON:
        if (!C.nbuf) {
            return -1;
        }
        C.streaming = 1;
        C.period_us = 1000000 / C.mode->fps;
        host_stats.cap_streaming = 1;
        host_timer_set(HOST_TIMER_SENSOR, host_now_us() + C.period_us, sensor_tick);
        return 0;
    case VIDIOC_STREAMOFF:
        C.streaming = 0;
        host_stats.cap_streaming = 0;
        host_timer_set(HOST_TIMER_SENSOR, -1, NULL);
        for (int i = 0; i < C.nbuf; i++) {
            C.b[i].state = B_USER;
        }
        return 0;
    default:
        host_violation("capture device: ioctl 0x%lx not supported", req);
        return -1;
    }
}

/* ---- the JPEG encoder ----------------------------------------------------- */
static int jpeg_ioctl(unsigned long req, void *arg)
{
    switch (req) {
    case VIDIOC_S_DQBUF_TIMEOUT: {
        struct timeval *tv = arg;
        J.dq_timeout_us = (int64_t)tv->tv_sec * 1000000 + tv->tv_usec;
        return 0;
    }
    case VIDIOC_S_EXT_CTRLS: {
        struct v4l2_ext_controls *cs = arg;
        if (cs->count != 1 || cs->controls[0].id != V4L2_CID_JPEG_COMPRESSION_QUALITY ||
            cs->controls[0].value < 1 || cs->controls[0].value > 100) {
            return -1;
        }
        J.quality = cs->controls[0].value;
        return 0;
    }
    case VIDIOC_S_FMT: {
        struct v4l2_format *f = arg;
        if (J.out_req || J.in_req) {
            if ((int)f->fmt.pix.width != J.w || (int)f->fmt.pix.height != J.h) {
                host_violation("JPEG format size changed after buffers were requested");
                return -1;
            }
            return 0;
        }
        J.w = (int)f->fmt.pix.width;
        J.h = (int)f->fmt.pix.height;
        return 0;
    }
    case VIDIOC_G_FMT: {
        struct v4l2_format *f = arg;
        f->fmt.pix.width = (uint32_t)J.w;
        f->fmt.pix.height = (uint32_t)J.h;
        f->fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG;
        f->fmt.pix.sizeimage = 0;                /* as in this esp_video build */
        return 0;
    }
    case VIDIOC_REQBUFS: {
        struct v4l2_requestbuffers *r = arg;
        if (!J.w) {
            return -1;
        }
        if (r->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
            J.in_req = r->count != 0;
            return r->memory == V4L2_MEMORY_USERPTR ? 0 : -1;
        }
        if (r->count != 1 || r->memory != V4L2_MEMORY_MMAP || J.out) {
            return -1;
        }
        J.out_len = (size_t)J.w * J.h;
        J.out = malloc(J.out_len);
        J.out_req = 1;
        J.out_state = B_USER;
        return J.out ? 0 : -1;
    }
    case VIDIOC_QUERYBUF: {
        struct v4l2_buffer *b = arg;
        if (!J.out || b->index != 0) {
            return -1;
        }
        b->length = (uint32_t)J.out_len;
        b->m.offset = 0;
        return 0;
    }
    case VIDIOC_QBUF: {
        struct v4l2_buffer *b = arg;
        if (b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
            if (!J.in_req || !J.on_in) {
                host_violation("JPEG input queued before the encoder's input side was started");
                return -1;
            }
            if (J.in_state == 1) {
                /* After a failed dequeue the old input is still queued: every later frame fails here
                 * until the firmware restarts the pipeline. Without such a failure it is a firmware mistake. */
                if (!J.dq_failed) {
                    host_violation("JPEG input queued twice");
                }
                return -1;
            }
            J.in_ptr = (const uint8_t *)b->m.userptr;
            J.in_state = 1;
            for (int i = 0; i < C.nbuf; i++) {
                if (J.in_ptr == C.b[i].mem && C.b[i].state != B_USER) {
                    host_violation("JPEG input is capture buffer %d, which the sensor may overwrite (it was re-queued)", i);
                }
            }
            if (b->length < (uint32_t)J.w * J.h * 2) {
                host_violation("JPEG input is %u bytes, smaller than one %dx%d RGB565 frame", b->length, J.w, J.h);
            }
            return 0;
        }
        if (!J.out) {
            host_violation("JPEG output buffer queued before it was requested");
            return -1;
        }
        if (J.out_state == B_QUEUED) {
            host_violation("JPEG output buffer queued twice");
            return -1;
        }
        J.out_state = B_QUEUED;
        return 0;
    }
    case VIDIOC_DQBUF: {
        struct v4l2_buffer *b = arg;
        if (b->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
            if (J.in_state != 2) {
                return -1;
            }
            J.in_state = 0;
            b->index = 0;
            return 0;
        }
        if (!J.on_in || !J.on_out) {
            host_violation("JPEG DQBUF while the encoder is not streaming");
            return -1;
        }
        if (J.in_state != 1 || J.out_state != B_QUEUED) {
            host_sleep_us(J.dq_timeout_us);      /* nothing to encode, or nowhere to put it */
            return -1;
        }
        if (host_params.jpeg_queue_error > 0) {
            host_params.jpeg_queue_error--;
            J.dq_failed = 1;
            return -1;
        }
        uint32_t used = 0;
        if (host_params.jpeg_fail > 0) {
            host_params.jpeg_fail--;
        } else {
            memset(&host_event, 0, sizeof(host_event));
            host_event.kind = FW_EV_NEED_JPEG;
            host_event.width = J.w;
            host_event.height = J.h;
            host_event.quality = J.quality ? J.quality : 80;
            host_event.corrupt = ((uintptr_t)J.in_ptr % ALIGN) != 0;
            host_event.src = (uint8_t *)J.in_ptr;
            host_event.ptr = J.out;
            host_event.len = (uint32_t)J.out_len;
            host_event.seq = s_last_seq;
            host_event.t_exposure_us = s_last_t;
            if (host_event.corrupt) {
                host_violation("JPEG input is not %d-byte aligned: the hardware encoder corrupts such frames", ALIGN);
            }
            s_jpeg_len = 0;
            host_wait_for_host();
            used = s_jpeg_len <= J.out_len ? s_jpeg_len : 0;
            host_sleep_us(host_params.jpeg_us_per_mp * J.w * J.h / 1000000);
        }
        host_stats.jpeg_encodes++;
        J.out_used = used;
        J.out_state = B_USER;
        J.in_state = 2;
        J.seq = s_last_seq;
        J.t_exposure_us = s_last_t;
        b->index = 0;
        b->bytesused = used;
        b->flags = 0;
        return 0;
    }
    case VIDIOC_STREAMON:
    case VIDIOC_STREAMOFF: {
        int on = req == VIDIOC_STREAMON;
        if (*(int *)arg == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
            J.on_in = on;
        } else {
            J.on_out = on;
        }
        if (!on) {
            J.in_state = 0;
        }
        return 0;
    }
    default:
        host_violation("JPEG device: ioctl 0x%lx not supported", req);
        return -1;
    }
}

void host_jpeg_done(uint32_t len) { s_jpeg_len = len; }

/* Is `buf` the encoder's output buffer, mapped, and holding a finished frame the firmware still owns? */
int host_jpeg_frame_live(const uint8_t *buf, uint64_t *seq, int64_t *t_exposure_us)
{
    *seq = J.seq;
    *t_exposure_us = J.t_exposure_us;
    return J.open && buf == J.out && J.out_mapped && J.out_state == B_USER;
}

/* ---- file calls ----------------------------------------------------------- */
int host_open(const char *path, int flags, ...)
{
    (void)flags;
    int cap = strcmp(path, ESP_VIDEO_MIPI_CSI_DEVICE_NAME) == 0, jpeg = strcmp(path, ESP_VIDEO_JPEG_DEVICE_NAME) == 0;
    if (!s_inited || (!cap && !jpeg) || (cap && !host_params.sensor_present)) {
        return -1;
    }
    if ((cap && C.open) || (jpeg && J.open)) {
        host_violation("%s opened twice", path);
        return -1;
    }
    if (cap) {
        memset(&C, 0, sizeof(C));
        C.filling = -1;
        C.open = 1;
        return FD_CAP;
    }
    memset(&J, 0, sizeof(J));
    J.open = 1;
    return FD_JPEG;
}

int host_close(int fd)
{
    if (fd == FD_CAP && C.open) {
        if (C.streaming) {
            host_violation("capture device closed while streaming");
            cap_ioctl(VIDIOC_STREAMOFF, NULL);
        }
        for (int i = 0; i < C.nbuf; i++) {
            if (C.b[i].mapped) {
                host_violation("capture device closed with buffer %d still mapped", i);
                host_stats.live_maps--;
            }
        }
        cap_free_buffers();
        C.open = 0;
        return 0;
    }
    if (fd == FD_JPEG && J.open) {
        if (J.out_mapped) {
            host_violation("JPEG device closed with its output buffer still mapped");
            host_stats.live_maps--;
        }
        free(J.out);
        memset(&J, 0, sizeof(J));
        return 0;
    }
    host_violation("close of fd %d, which is not an open video device", fd);
    return -1;
}

int host_ioctl(int fd, unsigned long req, ...)
{
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    if (fd == FD_CAP && C.open) {
        return cap_ioctl(req, arg);
    }
    if (fd == FD_JPEG && J.open) {
        return jpeg_ioctl(req, arg);
    }
    host_violation("ioctl 0x%lx on fd %d, which is not an open video device", req, fd);
    return -1;
}

void *host_mmap(void *addr, size_t len, int prot, int flags, int fd, long offset)
{
    (void)addr; (void)prot; (void)flags;
    if (fd == FD_CAP && C.open && offset >= 0 && offset < C.nbuf && len == C.b[offset].len && !C.b[offset].mapped) {
        C.b[offset].mapped = 1;
        host_stats.live_maps++;
        return C.b[offset].mem;
    }
    if (fd == FD_JPEG && J.open && J.out && offset == 0 && len == J.out_len && !J.out_mapped) {
        J.out_mapped = 1;
        host_stats.live_maps++;
        return J.out;
    }
    return MAP_FAILED;
}

int host_munmap(void *addr, size_t len)
{
    (void)len;
    for (int i = 0; i < C.nbuf; i++) {
        if (C.open && C.b[i].mapped && C.b[i].mem == addr) {
            C.b[i].mapped = 0;
            host_stats.live_maps--;
            return 0;
        }
    }
    if (J.open && J.out_mapped && J.out == addr) {
        J.out_mapped = 0;
        host_stats.live_maps--;
        return 0;
    }
    host_violation("munmap of %p, which is not a mapped video buffer", addr);
    return -1;
}
