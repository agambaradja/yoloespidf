#ifndef ELOQUENT_EXTRA_ESP32_MULTIPROCESSING_MUTEX
#define ELOQUENT_EXTRA_ESP32_MULTIPROCESSING_MUTEX

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

namespace Eloquent {
    namespace Extra {
        namespace Esp32 {
            namespace Multiprocessing {
                /**
                 * Mutex for concurrent access to resource
                 */
                class Mutex {
                public:
                    const char *name;
                    SemaphoreHandle_t mutex;

                    /**
                     * Constructor
                     */
                    Mutex(const char *name_) :
                        name(name_),
                        mutex(NULL),
                        _ok(true) {
                    }

                    /**
                     * Check if mutex is in a good state
                     */
                    bool isOk() const {
                        return _ok;
                    }

                    /**
                     * Run function with mutex
                     */
                    template<typename Callback>
                    bool threadsafe(Callback callback, size_t timeout = 0) {
                        TickType_t ticks = timeout / portTICK_PERIOD_MS;

                        if (timeout == 0) {
                            ticks = portMAX_DELAY;
                        }

                        if (mutex == NULL) {
                            ESP_LOGI("Mutex", "Creating mutex %s", name);
                            mutex = xSemaphoreCreateMutex();
                        }

                        if (mutex == NULL) {
                            ESP_LOGE("Mutex", "Cannot create mutex %s", name);
                            return false; 
                        }

                        if (xSemaphoreTake(mutex, ticks) != pdTRUE) {
                            ESP_LOGW("Mutex", "Cannot acquire mutex %s within timeout", name);
                            return (_ok = false);
                        }

                        callback();
                        xSemaphoreGive(mutex);

                        return (_ok = true);
                    }

                protected:
                    bool _ok;
                };
            }
        }
    }
}

#endif
