#include <cstdio>
#include <cstdlib>
#include <esp_system.h>
#include <esp_camera.h>
#include <esp_log.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "src\edge-impulse-sdk\classifier\ei_run_classifier.h"
#include <driver/uart.h>

#define TAG "EdgeImpulseDemo"

#define MODEL_XIAO_ESP32S3

#ifdef MODEL_XIAO_ESP32S3
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13
#else
#error "Camera model not selected"
#endif

#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

#ifndef RX
#define RX 44
#endif

#ifndef TX
#define TX 43
#endif


static bool debug_nn = false;
static bool is_initialised = false;
uint8_t* snapshot_buf;

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20 * 1000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

bool ei_camera_init();
void ei_camera_deinit();
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr);

uint32_t cx;
uint8_t data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, 'n', 0x00};

extern "C" void app_main() {
    const uart_port_t uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TX, RX, -1, -1);
    uart_driver_install(uart_num, 1024 * 2 , 0, 0, NULL, 0);

    ESP_LOGI(TAG, "Edge Impulse Inferencing Demo");
    if (!ei_camera_init()) {
        ESP_LOGE(TAG, "Failed to initialize Camera!");
        return;
    } else {
        ESP_LOGI(TAG, "Camera initialized");
    }

    ESP_LOGI(TAG, "Starting continuous inference in 2 seconds...");
    vTaskDelay(10 / portTICK_PERIOD_MS);

    while (true) {
        vTaskDelay(5 / portTICK_PERIOD_MS);

        snapshot_buf = static_cast<uint8_t*>(malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE));

        if (snapshot_buf == nullptr) {
            ESP_LOGE(TAG, "Failed to allocate snapshot buffer!");
            continue;
        }

        ei::signal_t signal;
        signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
        signal.get_data = &ei_camera_get_data;

        if (!ei_camera_capture(EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf)) {
            ESP_LOGE(TAG, "Failed to capture image");
            free(snapshot_buf);
            continue;
        }

        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "Failed to run classifier (%d)", err);
            free(snapshot_buf);
            continue;
        }

        ESP_LOGI(TAG, "Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): ",
                 result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
        ESP_LOGI(TAG, "Object detection bounding boxes:");
        for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) {
                continue;
            }

            cx = bb.x + bb.width / 2;

            ESP_LOGI(TAG, "  %s (%f) [ x: %u, y: %u, width: %u, height: %u , cx: %u]",
                    bb.label, bb.value, bb.x, bb.y, bb.width, bb.height, cx);

            uart_write_bytes(UART_NUM_0, data, sizeof(data));
        }
#else
        ESP_LOGI(TAG, "Predictions:");
        for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            ESP_LOGI(TAG, "  %s: %.5f", ei_classifier_inferencing_categories[i], result.classification[i].value);
        }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY
        ESP_LOGI(TAG, "Anomaly prediction: %.3f", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
        ESP_LOGI(TAG, "Visual anomalies:");
        for (uint32_t i = 0; i < result.visual_ad_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
            if (bb.value == 0) {
                continue;
            }
            ESP_LOGI(TAG, "  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]",
                    bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
        }
#endif

        free(snapshot_buf);
    }
}

bool ei_camera_init() {
    if (is_initialised) return true;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);
        s->set_brightness(s, 1);
        s->set_saturation(s, 0);
    }

    is_initialised = true;
    return true;
}

void ei_camera_deinit() {
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed");
        return;
    }
    is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t* out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ESP_LOGE(TAG, "Camera is not initialized");
        return false;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        return false;
    }

    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);
    esp_camera_fb_return(fb);

    if (!converted) {
        ESP_LOGE(TAG, "Conversion failed");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
            out_buf,
            EI_CAMERA_RAW_FRAME_BUFFER_COLS,
            EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
            out_buf,
            img_width,
            img_height
        );
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float* out_ptr) {
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];
        out_ptr_ix++;
        pixel_ix += 3;
        pixels_left--;
    }
    return 0;
}
