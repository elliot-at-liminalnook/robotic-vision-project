/*
 * Stand-in for Espressif's usb_device_uvc 1.2.0~1 (and the TinyUSB under it).
 *
 * video_task() below follows the component's own task line for line (frame
 * pacing on the 10 ms tick, one frame in flight, "continue" after a failed
 * fb_get), because that loop decides the frame rate. What is replaced is the
 * USB side:
 *   - the host program plays the PC: host_usb_commit (the PC selects a frame
 *     size and starts the stream), host_usb_stream(0) (the PC closes the
 *     stream: no callback reaches the firmware) and host_usb_suspend;
 *   - a frame takes len / usb_bytes_per_s to send, then arrives at the host
 *     program as FW_EV_FRAME_OUT;
 *   - a zero-length frame is refused and never completes, as TinyUSB does.
 * As in the component, start_cb runs on the TinyUSB task and blocks it.
 */
#include <string.h>
#include "host_internal.h"

char const *string_desc_arr[8] = {"", "robotic-vision-project", "P4 IMX708 Camera", "000000000000"};

static uvc_device_config_t s_cfg;
static uint32_t s_interval_ms;
static TaskHandle_t s_uvc_task, s_tusb_task;
static int s_streaming;
static struct { int kind, frame_index, fps; } s_cmd;       /* 1 = commit, 2 = suspend */
static uint32_t s_tx_len;
static uint64_t s_tx_seq;
static int64_t s_tx_t_exposure;

static uint32_t get_time_millis(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

static void usb_done(void)
{
    if (s_streaming) {
        memset(&host_event, 0, sizeof(host_event));
        host_event.kind = FW_EV_FRAME_OUT;
        host_event.ptr = s_cfg.uvc_buffer;
        host_event.len = s_tx_len;
        host_event.seq = s_tx_seq;
        host_event.t_exposure_us = s_tx_t_exposure;
        host_stats.frames_out++;
        host_event_post();
    } else {
        host_stats.usb_dropped++;          /* the PC closed the stream mid-frame */
    }
    xTaskNotifyGive(s_uvc_task);           /* tud_video_frame_xfer_complete_cb */
}

static void frame_xfer(uint32_t len)       /* tud_video_n_frame_xfer */
{
    if (len == 0) {
        host_stats.zero_length_frames++;
        host_violation("a zero-length frame was handed to USB: TinyUSB refuses it and the UVC task waits forever");
        return;
    }
    int64_t us = (int64_t)len * 1000000 / host_params.usb_bytes_per_s;
    us = (us / 125 + 1) * 125;             /* whole microframes */
    s_tx_len = len;
    host_timer_set(HOST_TIMER_USB, host_now_us() + us, usb_done);
}

static void video_task(void *arg)
{
    (void)arg;
    uint32_t start_ms = 0, already_start = 0, tx_busy = 0;
    uvc_fb_t *pic;

    while (1) {
        if (!s_streaming) {
            already_start = 0;
            tx_busy = 0;
            vTaskDelay(1);
            continue;
        }
        if (!already_start) {
            already_start = 1;
            start_ms = get_time_millis();
        }
        uint32_t cur = get_time_millis();
        if (cur - start_ms < s_interval_ms) {
            vTaskDelay(1);
            continue;
        }
        if (tx_busy) {
            uint32_t xfer_done = ulTaskNotifyTake(pdTRUE, 1);
            if (xfer_done == 0) {
                continue;
            }
            tx_busy = 0;
        }
        start_ms += s_interval_ms;
        pic = s_cfg.fb_get_cb(s_cfg.cb_ctx);
        if (!pic) {
            continue;                       /* "Failed to capture picture" */
        }
        if (pic->len > s_cfg.uvc_buffer_size) {
            ESP_LOGW("usbd_uvc", "frame size is too big, dropping frame");
            s_cfg.fb_return_cb(pic, s_cfg.cb_ctx);
            continue;
        }
        if (!host_jpeg_frame_live(pic->buf, &s_tx_seq, &s_tx_t_exposure)) {
            host_violation("fb_get returned a frame that is not in a live, mapped JPEG buffer (use after teardown)");
        } else {
            memcpy(s_cfg.uvc_buffer, pic->buf, pic->len);
        }
        uint32_t len = (uint32_t)pic->len;
        s_cfg.fb_return_cb(pic, s_cfg.cb_ctx);
        tx_busy = 1;
        frame_xfer(len);
    }
}

static void tusb_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (!s_cmd.kind) {
            host_block(&s_cmd, -1);
            continue;
        }
        if (s_cmd.kind == 1) {              /* tud_video_commit_cb, then the PC selects the streaming interface */
            int i = s_cmd.frame_index - 1;
            s_interval_ms = 1000u / (uint32_t)s_cmd.fps;
            esp_err_t ret = s_cfg.start_cb(UVC_FORMAT_JPEG, UVC_FRAMES_INFO[0][i].width, UVC_FRAMES_INFO[0][i].height,
                                           UVC_FRAMES_INFO[0][i].rate, s_cfg.cb_ctx);
            host_stats.commit_result = ret;
            host_stats.commits_done++;
            s_streaming = ret == ESP_OK;
        } else {                            /* tud_suspend_cb */
            s_streaming = 0;
            s_cfg.stop_cb(s_cfg.cb_ctx);
        }
        host_stats.uvc_streaming = s_streaming;
        s_cmd.kind = 0;
    }
}

esp_err_t uvc_device_config(int index, uvc_device_config_t *config)
{
    if (index != 0 || !config || !config->start_cb || !config->fb_get_cb || !config->fb_return_cb ||
        !config->stop_cb || !config->uvc_buffer || !config->uvc_buffer_size) {
        return ESP_ERR_INVALID_ARG;
    }
    s_cfg = *config;
    s_interval_ms = 1000 / CONFIG_UVC_CAM1_FRAMERATE;
    return ESP_OK;
}

esp_err_t uvc_device_init(void)
{
    xTaskCreate(tusb_task, "TinyUSB", 4096, NULL, CONFIG_UVC_TINYUSB_TASK_PRIORITY, &s_tusb_task);
    xTaskCreate(video_task, "UVC", 4096, NULL, CONFIG_UVC_CAM1_TASK_PRIORITY, &s_uvc_task);
    return ESP_OK;
}

/* ---- the PC's side -------------------------------------------------------- */
int host_usb_commit(int frame_index, int fps)
{
    if (s_cmd.kind || frame_index < 1 || frame_index > UVC_FRAME_NUM || fps < 1) {
        return -1;                          /* the TinyUSB task is still busy with the last request */
    }
    s_streaming = 0;                        /* a commit happens with the stream closed */
    host_stats.uvc_streaming = 0;
    s_cmd.kind = 1;
    s_cmd.frame_index = frame_index;
    s_cmd.fps = fps;
    host_wake(&s_cmd);
    return 0;
}

void host_usb_stream(int on)
{
    s_streaming = on;
    host_stats.uvc_streaming = on;
}

int host_usb_suspend(void)
{
    if (s_cmd.kind) {
        return -1;
    }
    s_cmd.kind = 2;
    host_wake(&s_cmd);
    return 0;
}
