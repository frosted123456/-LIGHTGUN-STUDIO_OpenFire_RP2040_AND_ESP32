#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum { PIXFORMAT_GRAYSCALE, PIXFORMAT_JPEG } pixformat_t;
typedef enum { FRAMESIZE_HQVGA, FRAMESIZE_QVGA } framesize_t;
typedef struct { int pin_pwdn,pin_reset,pin_xclk,pin_sccb_sda,pin_sccb_scl,pin_d7,pin_d6,pin_d5,pin_d4,pin_d3,pin_d2,pin_d1,pin_d0,pin_vsync,pin_href,pin_pclk; int xclk_freq_hz; int ledc_timer, ledc_channel; pixformat_t pixel_format; framesize_t frame_size; int jpeg_quality; size_t fb_count; int fb_location; int grab_mode; } camera_config_t;
typedef struct { uint8_t* buf; size_t len; size_t width, height; pixformat_t format; } camera_fb_t;
typedef int esp_err_t;
#define ESP_OK 0
#define CAMERA_FB_IN_PSRAM 0
#define CAMERA_FB_IN_DRAM 1
#define CAMERA_GRAB_WHEN_EMPTY 0
#define CAMERA_GRAB_LATEST 1
#define LEDC_TIMER_0 0
#define LEDC_CHANNEL_0 0
typedef struct _sensor sensor_t;
struct _sensor {
    int (*set_reg)(sensor_t*, int, int, int); int (*get_reg)(sensor_t*, int, int);
    int (*set_exposure_ctrl)(sensor_t*,int); int (*set_gain_ctrl)(sensor_t*,int);
    int (*set_agc_gain)(sensor_t*,int); int (*set_aec_value)(sensor_t*,int);
    int (*set_whitebal)(sensor_t*,int); int (*set_special_effect)(sensor_t*,int);
    int (*set_lenc)(sensor_t*,int); int (*set_raw_gma)(sensor_t*,int);
    int (*set_bpc)(sensor_t*,int); int (*set_wpc)(sensor_t*,int);
    int (*set_dcw)(sensor_t*,int); int (*set_gainceiling)(sensor_t*,int);
    int (*set_brightness)(sensor_t*,int); int (*set_contrast)(sensor_t*,int);
    int (*set_saturation)(sensor_t*,int); int (*set_hmirror)(sensor_t*,int);
    int (*set_vflip)(sensor_t*,int); int (*set_framesize)(sensor_t*,framesize_t); int (*set_aec2)(sensor_t*,int); int (*set_ae_level)(sensor_t*,int); int (*set_awb_gain)(sensor_t*,int);
};
extern "C" {
esp_err_t esp_camera_init(const camera_config_t*);
camera_fb_t* esp_camera_fb_get(void);
void esp_camera_fb_return(camera_fb_t*);
sensor_t* esp_camera_sensor_get(void);
}
