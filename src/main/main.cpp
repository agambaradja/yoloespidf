#include <espcamfinal_inferencing.h>
#include <eloquent_esp32cam.h>
#include <eloquent_esp32cam/edgeimpulse/yolo.h>
#include <esp_log.h>
#include <driver/uart.h>
#include <esp_heap_caps.h>

#ifndef RX
#define RX 44
#endif

#ifndef TX
#define TX 43
#endif

using eloq::camera;
using eloq::ei::yolo;

static const char *TAG = "main";
char pos;

void camera_task(void *pvParameter) {
    while (true) {
        if (!camera.capture().isOk()) {
            ESP_LOGE(TAG, "Failed to capture frame: %s", camera.exception.toString().c_str());
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to avoid tight loop on failure
            continue;
        }

        ESP_LOGI(TAG, "Total heap: %d", heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
        ESP_LOGI(TAG, "Free heap: %d", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
        ESP_LOGI(TAG, "Total PSRAM: %d", heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI(TAG, "Free PSRAM: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

        if (!yolo.run().isOk()) {
            ESP_LOGE(TAG, "YOLO inference failed: %s", yolo.exception.toString().c_str());
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to avoid tight loop on failure
            continue;
        }

        if (!yolo.foundAnyObject()) {
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay if no objects found
            continue;
        }

        if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) {
            pos = 'l';
        } else if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3) {
            pos = 'c';
        } else {
            pos = 'r';
        }

        uint8_t esp_data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, static_cast<uint8_t>(pos), 0x00};
        uart_write_bytes(UART_NUM_0, (const char *)esp_data, sizeof(esp_data));

        if (yolo.count() > 1) {
            yolo.forEach([](int i, bbox_t bbox) {
                if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) pos = 'l';
                else if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3) pos = 'c';
                else pos = 'r';
                uint8_t esp_data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, static_cast<uint8_t>(pos), 0x00};
                uart_write_bytes(UART_NUM_0, (const char *)esp_data, sizeof(esp_data));
            });
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay between captures
    }
}

extern "C" void app_main() {
    esp_log_level_set(TAG, ESP_LOG_VERBOSE);

    // Initialize UART0 for debugging
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
    uart_set_pin(uart_num, TX, RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, 1024 * 2, 0, 0, NULL, 0);

    // Initialize the camera
    camera.brownout.disable();
    camera.resolution.yolo();

    while (!camera.begin().isOk()) {
        ESP_LOGE(TAG, "Camera initialization failed: %s", camera.exception.toString().c_str());
    }
    ESP_LOGI(TAG, "Camera initialized successfully");

    // Create the camera task
    xTaskCreate(&camera_task, "camera_task", 8192, NULL, 5, NULL);
}
