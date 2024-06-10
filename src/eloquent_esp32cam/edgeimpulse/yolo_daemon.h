#ifndef ELOQUENT_ESP32CAM_EDGEIMPULSE_yolo_DAEMON_H
#define ELOQUENT_ESP32CAM_EDGEIMPULSE_yolo_DAEMON_H

#include <functional>
#include "../camera/camera.h"
#include "../extra/esp32/multiprocessing/thread.h"
#include "./bbox.h"

#include "esp_log.h" // Include ESP-IDF logging library

using eloq::camera;
using eloq::ei::bbox_t;
using Eloquent::Extra::Esp32::Multiprocessing::Thread;
using OnObjectCallback = std::function<void(uint8_t, bbox_t&)>;
using OnNothingCallback = std::function<void()>;

namespace Eloquent {
    namespace Esp32cam {
        namespace EdgeImpulse {
            /**
             * Run yolo in background
             */
            template<typename T>
            class yoloDaemon {
            public:
                Thread thread;

                /**
                 * Constructor
                 */
                yoloDaemon(T *yolo) :
                    thread("yolo"),
                    _yolo(yolo),
                    _numListeners(0) {
                }

                /**
                 * Run function when an object is detected
                 */
                bool whenYouSeeAny(OnObjectCallback callback) {
                    return whenYouSee("*", callback);
                }

                /**
                 * Run function when nothing is detected
                 */
                void whenYouDontSeeAnything(OnNothingCallback callback) {
                    _onNothing = callback;
                }

                /**
                 * Run function when a specific object is detected
                 */
                bool whenYouSee(std::string label, OnObjectCallback callback) {
                    if (_numListeners >= EI_CLASSIFIER_LABEL_COUNT + 1) {
                        ESP_LOGE("yolo daemon", "Max number of listeners reached");
                        return false;
                    }

                    _callbacks[_numListeners].label = label;
                    _callbacks[_numListeners].callback = callback;
                    _numListeners += 1;

                    return true;
                }

                /**
                 * Start yolo in background
                 */
                void start() {
                    thread
                        .withArgs((void*) this)
                        .withStackSize(6000)
                        .withPriority(17) // Adjust priority as needed
                        .run([](void *args) {
                            yoloDaemon *self = (yoloDaemon*) args;

                            vTaskDelay(3000 / portTICK_PERIOD_MS); // Use vTaskDelay instead of delay

                            while (true) {
                                vTaskDelay(1); // Use vTaskDelay instead of delay

                                if (!camera.capture().isOk())
                                    continue;

                                if (!self->_yolo->run().isOk())
                                    continue;

                                if (!self->_yolo->foundAnyObject()) {
                                    if (self->_onNothing)
                                        self->_onNothing();

                                    continue;
                                }

                                self->_yolo->forEach([&self](int i, bbox_t& bbox) {
                                    // Run specific label callback
                                    for (uint8_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT + 1; i++) {
                                        std::string label = self->_callbacks[i].label;

                                        if (label == "*" || label == bbox.label)
                                            self->_callbacks[i].callback(i, bbox);
                                    }
                                });
                            }
                        });
                }

            protected:
                T *_yolo;
                uint8_t _numListeners;
                OnNothingCallback _onNothing;
                struct {
                    std::string label;
                    OnObjectCallback callback;
                } _callbacks[EI_CLASSIFIER_LABEL_COUNT + 1];

            };
        }
    }
}

#endif
