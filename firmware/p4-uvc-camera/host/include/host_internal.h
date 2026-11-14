/* Shared between the stand-ins (rtos.c, video_fake.c, uvc_fake.c) and host_api.c. Not seen by the firmware. */
#pragma once
#include "host_idf.h"

/* Settings the host program can change at any time (fw_set). Times are placeholders until measured on a board. */
typedef struct {
    int64_t init_us;            /* esp_video_init: sensor detect, ISP and JPEG bring-up */
    int64_t deinit_us;
    int64_t sensor_mode_us;     /* VIDIOC_S_SENSOR_FMT */
    int64_t readout_us;         /* exposure -> the frame is in a capture buffer (sensor readout, CSI, ISP) */
    int64_t jpeg_us_per_mp;     /* hardware JPEG encode time per megapixel */
    int64_t usb_bytes_per_s;    /* isochronous endpoint: 1023 bytes per 125 us microframe */
    int64_t sensor_present;     /* 0: no camera answers on the ribbon cable */
    int64_t init_fail;          /* esp_video_init fails this many times */
    int64_t misalign;           /* capture buffers come back not 128-byte aligned */
    int64_t sensor_stalled;     /* the sensor stops producing frames */
    int64_t jpeg_fail;          /* the next N encodes produce 0 bytes */
    int64_t jpeg_queue_error;   /* the next N encoder dequeues fail */
} host_params_t;
extern host_params_t host_params;

typedef struct {
    int64_t sensor_frames, sensor_dropped, jpeg_encodes, frames_out, usb_dropped, zero_length_frames;
    int64_t live_allocs, live_maps, cap_streaming, video_inited, uvc_streaming, commit_result, commits_done;
} host_stats_t;
extern host_stats_t host_stats;

extern fw_event_t host_event;          /* the event handed to the host program */
void host_event_post(void);            /* from a device timer: return to the host with host_event */
void host_resume_from_host(void);
void host_lock(void);
void host_unlock(void);
int host_run(int64_t until_us);
int host_task_summary(char *buf, int n);
int host_log_count(void);
const char *host_log_line(int i);
int host_violation_count(void);
const char *host_violation_line(int i);
int host_sem_held(SemaphoreHandle_t s);

/* video_fake.c */
void host_frame_filled(void);
void host_jpeg_done(uint32_t len);
int host_jpeg_frame_live(const uint8_t *buf, uint64_t *seq, int64_t *t_exposure_us);
void host_set_mac(const uint8_t *mac);

/* uvc_fake.c */
int host_usb_commit(int frame_index, int fps);
void host_usb_stream(int on);
int host_usb_suspend(void);
