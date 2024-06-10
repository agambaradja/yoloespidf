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
uint8_t pos = 'n';
uint8_t esp_data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, pos, 0x00};

extern "C" void app_main() {
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
    uart_set_pin(uart_num, TX, RX, -1, -1);
    uart_driver_install(uart_num, 1024 * 2 , 0, 0, NULL, 0);
    camera.resolution.yolo();

    while (!camera.begin().isOk());
    
    //ESP_LOGI(TAG, "Camera initialized successfully"); 

    while (true) {
        if (!camera.capture().isOk()) {
            // ESP_LOGE(TAG, "Failed to capture frame: %s", camera.exception.toString().c_str());
            vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to avoid tight loop on failure
            continue;
        }
         if (!yolo.run().isOk()) {
            // ESP_LOGE(TAG, "YOLO inference failed: %s", yolo.exception.toString().c_str());
            vTaskDelay(10 / portTICK_PERIOD_MS); // Delay to avoid tight loop on failure
            continue;
        }
         if (!yolo.foundAnyObject()) {
            pos = 'n';
            //ESP_LOGI(TAG, "pposisi: %c", pos);
            //uint8_t esp_data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, pos, 0x00};
            esp_data[5] = 'n';
            uart_write_bytes(UART_NUM_0, esp_data, sizeof(esp_data));
            vTaskDelay(10 / portTICK_PERIOD_MS); // Delay if no objects found
            continue;
        }

/*         if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) pos = 'l'; //esp_data[5] = 'l'; 
        else if (yolo.first.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3)  pos = 'c'; //esp_data[5] = 'c';
        else pos = 'r';//esp_data[5] = 'r';  */

        if (yolo.first.cx <= 13){
            pos = 'l';
        } else if (yolo.first.cx <= 19){
            pos = 'c';
        } else {
            pos = 'r';
        }

        //ESP_LOGI(TAG, "pposisi: %c", pos);
        //ESP_LOGI(TAG, "cx: %u", yolo.first.cx);

        esp_data[5] = pos;
        uart_write_bytes(UART_NUM_0, esp_data, sizeof(esp_data));

/*          if (yolo.count() > 1) {
            yolo.forEach([](int i, bbox_t bbox) {
                if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH / 3) esp_data[5] = 'l'; //pos = 'l';
                else if (bbox.cx <= EI_CLASSIFIER_INPUT_WIDTH * 2 / 3)  esp_data[5] = 'c'; //pos = 'c';
                else esp_data[5] = 'r'; //pos = 'r';
                ESP_LOGI(TAG, "pposisi: %c", pos);
                esp_data[5] = pos;
                //uint8_t esp_data[7] = {0x5A, 0x9F, 0x3A, 0x41, 0x6F, pos, 0x00};
                uart_write_bytes(UART_NUM_0, esp_data, sizeof(esp_data));
            });
        }  */ 
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay between captures
    }    
}