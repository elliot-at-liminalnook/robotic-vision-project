/*
 * Stand-ins for the ESP-IDF, esp_video, usb_device_uvc and IMX708 driver
 * declarations that main/uvc_camera_main.c uses, so the firmware source
 * compiles unchanged on a PC. Implemented in ../rtos.c, ../video_fake.c and
 * ../uvc_fake.c. Only what the firmware touches is declared.
 *
 * Checked by hand against esp_video 2.5.0, usb_device_uvc 1.2.0~1 and the
 * vendored IMX708 driver; not generated from them.
 */
#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/time.h>
#include "sdkconfig.h"

/* ---- esp_err / esp_log -------------------------------------------------- */
typedef int esp_err_t;
#define ESP_OK                 0
#define ESP_FAIL               (-1)
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_NOT_SUPPORTED  0x106
#define ESP_ERR_TIMEOUT        0x107
const char *esp_err_to_name(esp_err_t err);
void host_log(char level, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
void host_violation(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define ESP_LOGE(tag, ...) host_log('E', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) host_log('W', tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) host_log('I', tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) host_log('D', tag, __VA_ARGS__)
#define ESP_ERROR_CHECK(x) do { esp_err_t e_ = (x); if (e_ != ESP_OK) host_violation("ESP_ERROR_CHECK failed: %s (the real firmware aborts here)", #x); } while (0)

/* ---- FreeRTOS (deterministic, virtual time; rtos.c) --------------------- */
typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef struct host_task *TaskHandle_t;
typedef struct host_sem *SemaphoreHandle_t;
typedef struct host_queue *QueueHandle_t;
typedef void (*TaskFunction_t)(void *);
#define pdTRUE         1
#define pdFALSE        0
#define pdPASS         1
#define portMAX_DELAY  ((TickType_t)0xffffffffu)
#define configTICK_RATE_HZ CONFIG_FREERTOS_HZ
#define pdMS_TO_TICKS(ms) ((TickType_t)(((uint64_t)(ms) * configTICK_RATE_HZ) / 1000u))
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack, void *arg, UBaseType_t prio, TaskHandle_t *out);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
BaseType_t xTaskNotifyGive(TaskHandle_t t);
SemaphoreHandle_t xSemaphoreCreateMutex(void);
SemaphoreHandle_t xSemaphoreCreateBinary(void);
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t ticks);
BaseType_t xSemaphoreGive(SemaphoreHandle_t s);
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t ticks);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks);

/* Scheduler internals used by the device stand-ins. */
int64_t host_now_us(void);
void host_sleep_us(int64_t us);                       /* the calling task takes this long */
int host_block(const void *obj, int64_t timeout_us);  /* 1 = woken by host_wake, 0 = timed out; <0 = forever */
int host_wake(const void *obj);                       /* wake the best waiter on obj; 1 if there was one */
void host_wait_for_host(void);                        /* hand an event to the host program and wait for it */
typedef void (*host_timer_fn)(void);
void host_timer_set(int slot, int64_t at_us, host_timer_fn fn);   /* at_us < 0 cancels */
#define HOST_TIMER_SENSOR 0
#define HOST_TIMER_USB    1

/* ---- small ESP-IDF pieces ----------------------------------------------- */
#define ESP_MAC_BASE 0
esp_err_t esp_read_mac(uint8_t *mac, int type);
int64_t esp_timer_get_time(void);
#define MALLOC_CAP_SPIRAM 1
#define MALLOC_CAP_8BIT   2
void *heap_caps_malloc(size_t size, uint32_t caps);
void *heap_caps_aligned_alloc(size_t align, size_t size, uint32_t caps);
void heap_caps_free(void *p);

/* ---- esp_video ---------------------------------------------------------- */
typedef struct { int port, scl_pin, sda_pin; } esp_video_init_i2c_config_t;
typedef struct { bool init_sccb; esp_video_init_i2c_config_t i2c_config; uint32_t freq; } esp_video_init_sccb_config_t;
typedef struct { esp_video_init_sccb_config_t sccb_config; int reset_pin, pwdn_pin; } esp_video_init_csi_config_t;
typedef struct { esp_video_init_sccb_config_t sccb_config; int reset_pin, pwdn_pin, signal_pin; } esp_video_init_cam_motor_config_t;
typedef struct { const esp_video_init_csi_config_t *csi; const esp_video_init_cam_motor_config_t *cam_motor; } esp_video_init_config_t;
esp_err_t esp_video_init(const esp_video_init_config_t *config);
esp_err_t esp_video_deinit(void);
#define ESP_VIDEO_MIPI_CSI_DEVICE_NAME "/dev/video0"
#define ESP_VIDEO_JPEG_DEVICE_NAME     "/dev/video10"
typedef enum { ESP_VIDEO_ISP_PIPELINE_AGC_DISABLE = 0, ESP_VIDEO_ISP_PIPELINE_AGC_ENABLE = 1 } esp_video_isp_pipeline_agc_status_t;
esp_err_t esp_video_isp_pipeline_set_agc_status(esp_video_isp_pipeline_agc_status_t status);

/* ---- IMX708 driver ------------------------------------------------------ */
typedef struct { const char *name; uint16_t width, height; uint8_t fps; } esp_cam_sensor_format_t;
const esp_cam_sensor_format_t *imx708_format_by_size(uint16_t width, uint16_t height);

/* ---- V4L2 subset (esp_video's ioctl interface) -------------------------- */
enum { VIDIOC_S_FMT = 0x5601, VIDIOC_G_FMT, VIDIOC_REQBUFS, VIDIOC_QUERYBUF, VIDIOC_QBUF, VIDIOC_DQBUF,
       VIDIOC_STREAMON, VIDIOC_STREAMOFF, VIDIOC_S_EXT_CTRLS, VIDIOC_QUERY_EXT_CTRL,
       VIDIOC_S_SENSOR_FMT, VIDIOC_S_DQBUF_TIMEOUT };
enum { V4L2_BUF_TYPE_VIDEO_CAPTURE = 1, V4L2_BUF_TYPE_VIDEO_OUTPUT = 2 };
enum { V4L2_MEMORY_MMAP = 1, V4L2_MEMORY_USERPTR = 2 };
#define V4L2_BUF_FLAG_ERROR 0x40
#define V4L2_PIX_FMT_RGB565 0x50424752u   /* 'RGBP' */
#define V4L2_PIX_FMT_JPEG   0x4745504Au   /* 'JPEG' */
enum { V4L2_CID_USER_CLASS = 0x980001, V4L2_CID_GAIN = 0x980913, V4L2_CID_HFLIP = 0x980914, V4L2_CID_VFLIP = 0x980915,
       V4L2_CID_CAMERA_CLASS = 0x9a0001, V4L2_CID_EXPOSURE_ABSOLUTE = 0x9a0902, V4L2_CID_FOCUS_ABSOLUTE = 0x9a090a,
       V4L2_CID_JPEG_CLASS = 0x9d0001, V4L2_CID_JPEG_COMPRESSION_QUALITY = 0x9d0903 };
struct v4l2_pix_format { uint32_t width, height, pixelformat, sizeimage; };
struct v4l2_format { uint32_t type; struct { struct v4l2_pix_format pix; } fmt; };
struct v4l2_requestbuffers { uint32_t count, type, memory; };
struct v4l2_buffer { uint32_t index, type, bytesused, flags, memory; union { uint32_t offset; unsigned long userptr; } m; uint32_t length; };
struct v4l2_ext_control { uint32_t id; int32_t value; };
struct v4l2_ext_controls { uint32_t ctrl_class, count; struct v4l2_ext_control *controls; };
struct v4l2_query_ext_ctrl { uint32_t id; int64_t minimum, maximum; };

/* ---- usb_device_uvc ----------------------------------------------------- */
typedef enum { UVC_FORMAT_JPEG, UVC_FORMAT_H264 } uvc_format_t;
typedef struct { uint8_t *buf; size_t len; size_t width; size_t height; uvc_format_t format; struct timeval timestamp; } uvc_fb_t;
typedef esp_err_t (*uvc_input_start_cb_t)(uvc_format_t format, int width, int height, int rate, void *cb_ctx);
typedef uvc_fb_t *(*uvc_input_fb_get_cb_t)(void *cb_ctx);
typedef void (*uvc_input_fb_return_cb_t)(uvc_fb_t *fb, void *cb_ctx);
typedef void (*uvc_input_stop_cb_t)(void *cb_ctx);
typedef struct {
    uint8_t *uvc_buffer; uint32_t uvc_buffer_size;
    uvc_input_start_cb_t start_cb; uvc_input_fb_get_cb_t fb_get_cb; uvc_input_fb_return_cb_t fb_return_cb; uvc_input_stop_cb_t stop_cb;
    void *cb_ctx;
} uvc_device_config_t;
esp_err_t uvc_device_config(int index, uvc_device_config_t *config);
esp_err_t uvc_device_init(void);
static const struct { int width, height, rate; } UVC_FRAMES_INFO[][4] = {{
    {CONFIG_UVC_CAM1_FRAMESIZE_WIDTH, CONFIG_UVC_CAM1_FRAMESIZE_HEIGT, CONFIG_UVC_CAM1_FRAMERATE},
    {CONFIG_UVC_MULTI_FRAME_WIDTH_1, CONFIG_UVC_MULTI_FRAME_HEIGHT_1, CONFIG_UVC_MULTI_FRAME_FPS_1},
    {CONFIG_UVC_MULTI_FRAME_WIDTH_2, CONFIG_UVC_MULTI_FRAME_HEIGHT_2, CONFIG_UVC_MULTI_FRAME_FPS_2},
    {CONFIG_UVC_MULTI_FRAME_WIDTH_3, CONFIG_UVC_MULTI_FRAME_HEIGHT_3, CONFIG_UVC_MULTI_FRAME_FPS_3},
}};
#define UVC_FRAME_NUM 4

/* ---- what the host program (Python) sees: host_api ----------------------- */
enum { FW_EV_NONE = 0, FW_EV_NEED_FRAME = 1, FW_EV_NEED_JPEG = 2, FW_EV_FRAME_OUT = 3 };
typedef struct {
    int32_t kind;
    int32_t width, height;
    int32_t hflip, vflip;         /* NEED_FRAME: mirror the picture like the sensor would */
    int32_t wrong_colours;        /* NEED_FRAME: a flip was set after the sensor mode (Bayer phase mismatch) */
    int32_t exposure_us;          /* NEED_FRAME: fixed exposure in effect, or 0 for auto */
    int32_t gain_index;
    int32_t quality;              /* NEED_JPEG */
    int32_t corrupt;              /* NEED_JPEG: the encoder was given a misaligned buffer */
    uint32_t len;                 /* FRAME_OUT: bytes; NEED_*: capacity of ptr */
    uint8_t *ptr;                 /* NEED_FRAME: RGB565 to fill; NEED_JPEG: JPEG to fill; FRAME_OUT: the JPEG */
    uint8_t *src;                 /* NEED_JPEG: the RGB565 input */
    uint64_t seq;                 /* sensor frame number */
    int64_t t_exposure_us;        /* when that sensor frame was taken */
    int64_t t_us;                 /* now */
} fw_event_t;
