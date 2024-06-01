#ifndef ELOQUENT_EXTRA_TIME_BENCHMARK
#define ELOQUENT_EXTRA_TIME_BENCHMARK

#include "esp_timer.h" // Include ESP-IDF header for timing
#include "freertos/FreeRTOS.h" // Include FreeRTOS header for timing

namespace Eloquent {
    namespace Extra {
        namespace Time {
            /**
             * Run benchmark on code blocks
             */
            class Benchmark {
            public:

                /**
                 * Start timer
                 */
                void start() {
                    timeStart = esp_timer_get_time(); // Use ESP-IDF's esp_timer_get_time() instead of micros()
                }

                /**
                 * Stop timer
                 */
                size_t stop() {
                    elapsedInMicros = esp_timer_get_time() - timeStart; // Use ESP-IDF's esp_timer_get_time() instead of micros()

                    return millis();
                }

                /**
                 * Benchmark given function
                 */
                template<typename Callback>
                size_t benchmark(Callback callback) {
                    start();
                    callback();
                    
                    return stop();
                }

                /**
                 * Get elapsed time in millis
                 */
                inline size_t millis() {
                    return microseconds() / 1000;
                }

                /**
                 * Alias for millis()
                 */
                inline size_t ms() {
                    return millis();
                }

                /**
                 * Get elapsed time in micros
                 */
                inline size_t microseconds() {
                    return elapsedInMicros;
                }

                /**
                 * Alias for micros
                 */
                inline size_t us() {
                    return microseconds();
                }

                /**
                 * 
                 */
                template<typename Callback>
                void timeit(Callback callback) {
                    start();
                    callback();
                    stop();
                }

            protected:
                int64_t timeStart;
                int64_t elapsedInMicros;
            };
        }
    }
}

#endif
