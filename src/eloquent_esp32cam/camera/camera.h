#ifndef ELOQUENT_ESP32CAMERA_CAMERA_CAMERA
#define ELOQUENT_ESP32CAMERA_CAMERA_CAMERA

#include <esp_camera.h>
#include <esp_log.h>
#include "./brownout.h"
#include "./xclk.h"
#include "./resolution.h"
#include "./sensor.h"
#include "./pixformat.h"
#include "./rgb_565.h"
#include "../extra/exception.h"
#include "../extra/time/rate_limit.h"
#include "../extra/esp32/multiprocessing/mutex.h"

using Eloquent::Error::Exception;
using Eloquent::Extra::Esp32::Multiprocessing::Mutex;
using Eloquent::Extra::Time::RateLimit;

namespace Eloquent {
    namespace Esp32cam {
        namespace Camera {
            /**
             * Configure and use the camera,
             * Eloquent style
             */
            class Camera {
            public:
                camera_config_t config;
                camera_fb_t *frame;
                Brownout brownout;
                XCLK xclk;
                Resolution resolution;
                Sensor sensor;
                Pixformat pixformat;
                Exception exception;
                RateLimit rateLimit;
                Mutex mutex;
                Converter565<Camera> rgb565;

                /**
                 * Constructor
                 */
        Camera() :  frame(nullptr), // Initialize frame to nullptr
                    exception("Camera"),
                    mutex("Camera"),
                    rgb565(this)                
                {
                    // Initialize camera configuration with default values
                    config.ledc_channel = LEDC_CHANNEL_0;
                    config.ledc_timer = LEDC_TIMER_0;
                    config.pin_d0 = 15;
                    config.pin_d1 = 17;
                    config.pin_d2 = 18;
                    config.pin_d3 = 16;
                    config.pin_d4 = 14;
                    config.pin_d5 = 12;
                    config.pin_d6 = 11;
                    config.pin_d7 = 48;
                    config.pin_xclk = 10;
                    config.pin_pclk = 13;
                    config.pin_vsync = 38;
                    config.pin_href = 47;
                    config.pin_sccb_sda = 40;
                    config.pin_sccb_scl = 39;
                    config.pin_pwdn = -1;
                    config.pin_reset = -1;
                    config.xclk_freq_hz = 8 * 1000000;
                    config.pixel_format = PIXFORMAT_RGB565;

                    config.frame_size = FRAMESIZE_96X96;
                    config.fb_location = CAMERA_FB_IN_DRAM;
                    config.fb_count = 3;
                    config.grab_mode = CAMERA_GRAB_LATEST;
                }

                /**
                 * Initialize the camera
                 * @return
                 */
                Exception &begin() {
                    // Camera init
                    esp_err_t err = esp_camera_init(&config);
                    if (err != ESP_OK)
                        return exception.set("Camera init failed");

                    sensor.setFrameSize(resolution.framesize);

                    return exception.clear();
                }

                /**
                 * Capture new frame
                 */
                Exception &capture() {
                    if (!rateLimit)
                        return exception.soft().set("Too many requests for frame");

                    mutex.threadsafe([this]() {
                        free();
                        frame = esp_camera_fb_get();
                    }, 1000);

                    if (!mutex.isOk())
                        return exception.set("Cannot acquire mutex");

                    rateLimit.touch();

                    if (!hasFrame())
                        return exception.set("Cannot capture frame");

                    return exception.clear();
                }

                /**
                 * Release frame memory
                 */
                void free() {
                    if (frame != NULL) {
                        esp_camera_fb_return(frame);
                        frame = NULL;
                    }
                }

                /**
                 * Test if camera has a valid frame
                 */
                inline bool hasFrame() const {
                    return frame != NULL && frame->len > 0;
                }

                /**
                 * Get frame size in bytes
                 * @return
                 */
                inline size_t getSizeInBytes() const {
                    return hasFrame() ? frame->len : 0;
                }

                /**
                 * Save to given folder with automatic name
                 */
                template <typename Disk>
                Exception &saveTo(Disk &disk, const std::string &folder = "") {
                    return saveToAs(disk, folder, "");
                }

                /**
                 * Save to given folder with given name
                 */
                template <typename Disk>
                Exception &saveToAs(Disk &disk, const std::string &folder, const std::string &filename) {
                    if (!hasFrame())
                        return exception.set("No frame to save");

                    std::string file = filename;
                    if (file.empty())
                        file = disk.getNextFilename("jpg");

                    if (!folder.empty())
                        file = folder + '/' + file;

                    if (file[0] != '/')
                        file = '/' + file;

                    return disk.writeBinary(file, frame->buf, frame->len);
                }

            protected:
            };
        }
    }
}

namespace eloq {
    static Eloquent::Esp32cam::Camera::Camera camera;
}

#endif
