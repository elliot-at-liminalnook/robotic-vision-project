/*
 * The firmware, built for a PC, behind a small C interface for the host program
 * (sim/so101sim/camera_fw.py loads this with ctypes).
 *
 * The firmware source is included unchanged; including it here (rather than
 * compiling it separately) lets fw_get() read its private state for tests.
 */
#include "../main/uvc_camera_main.c"

#include "host_internal.h"

void fw_boot(const uint8_t *mac)
{
    host_lock();
    if (mac) {
        host_set_mac(mac);
    }
    app_main();
    host_unlock();
}

/* Run the firmware until virtual time until_us, or until it needs the host. Returns the event kind. */
int fw_run(int64_t until_us, fw_event_t *ev)
{
    host_lock();
    int has = host_run(until_us);
    if (has) {
        *ev = host_event;
    } else {
        ev->kind = FW_EV_NONE;
        ev->t_us = host_now_us();
    }
    host_unlock();
    return ev->kind;
}

void fw_frame_filled(void)          { host_lock(); host_frame_filled(); host_unlock(); }
void fw_jpeg_done(uint32_t len)     { host_lock(); host_jpeg_done(len); host_resume_from_host(); host_unlock(); }
int fw_host_commit(int index, int fps) { host_lock(); int r = host_usb_commit(index, fps); host_unlock(); return r; }
void fw_host_stream(int on)         { host_lock(); host_usb_stream(on); host_unlock(); }
int fw_host_suspend(void)           { host_lock(); int r = host_usb_suspend(); host_unlock(); return r; }

#define FIELDS(X) \
    X(init_us) X(deinit_us) X(sensor_mode_us) X(readout_us) X(jpeg_us_per_mp) X(usb_bytes_per_s) X(sensor_present) \
    X(init_fail) X(misalign) X(sensor_stalled) X(jpeg_fail) X(jpeg_queue_error)
#define STATS(X) \
    X(sensor_frames) X(sensor_dropped) X(jpeg_encodes) X(frames_out) X(usb_dropped) X(zero_length_frames) \
    X(live_allocs) X(live_maps) X(cap_streaming) X(video_inited) X(uvc_streaming) X(commit_result) X(commits_done)

int fw_set(const char *name, int64_t value)
{
#define X(f) if (strcmp(name, #f) == 0) { host_params.f = value; return 0; }
    FIELDS(X)
#undef X
    return -1;
}

int64_t fw_get(const char *name)
{
#define X(f) if (strcmp(name, #f) == 0) { return host_params.f; }
    FIELDS(X)
#undef X
#define X(f) if (strcmp(name, #f) == 0) { return host_stats.f; }
    STATS(X)
#undef X
    if (strcmp(name, "now_us") == 0)       { return host_now_us(); }
    if (strcmp(name, "cam_running") == 0)  { return s_cam.running; }       /* the firmware's own state */
    if (strcmp(name, "bad_frames") == 0)   { return s_cam.bad_frames; }
    if (strcmp(name, "staging") == 0)      { return s_cam.stage != NULL; }
    if (strcmp(name, "lock_held") == 0)    { return host_sem_held(s_lock); }
    if (strcmp(name, "frame_sizes") == 0)  { return UVC_FRAME_NUM; }
    return INT64_MIN;
}

int fw_frame_size(int index, int *width, int *height)
{
    if (index < 1 || index > (int)UVC_FRAME_NUM) {
        return -1;
    }
    *width = UVC_FRAMES_INFO[0][index - 1].width;
    *height = UVC_FRAMES_INFO[0][index - 1].height;
    return 0;
}

const char *fw_serial(void)              { return string_desc_arr[3]; }
int fw_log_count(void)                   { return host_log_count(); }
const char *fw_log_line(int i)           { return host_log_line(i); }
int fw_violation_count(void)             { return host_violation_count(); }
const char *fw_violation_line(int i)     { return host_violation_line(i); }
int fw_tasks(char *buf, int n)           { host_lock(); int r = host_task_summary(buf, n); host_unlock(); return r; }
