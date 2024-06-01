#ifndef ELOQUENT_ESP32CAM_EDGEIMPULSE_CLASSIFIER_H
#define ELOQUENT_ESP32CAM_EDGEIMPULSE_CLASSIFIER_H

#include "../extra/exception.h"
#include "../extra/time/benchmark.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

using Eloquent::Error::Exception;
using Eloquent::Extra::Time::Benchmark;
using ei::signal_t;

namespace Eloquent {
    namespace Esp32cam {
        namespace EdgeImpulse {
            /**
             * Base class for Edge Impulse classifiers
             */
            class Classifier {
            public:
                signal_t signal;
                ei_impulse_result_t result;
                EI_IMPULSE_ERROR error;
                Exception exception;
                Benchmark benchmark;
                struct {
                    size_t dsp;
                    size_t anomaly;
                    size_t classification;
                    size_t total;
                } timing;

                /**
                 * Constructor
                 */
                Classifier() :
                    exception("EI " EI_CLASSIFIER_PROJECT_NAME),
                    _isDebugEnabled(false) {
                    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
                }

                /**
                 * Enable or disable debug mode
                 */
                void debug(bool enabled = true) {
                    _isDebugEnabled = enabled;
                }

                /**
                 * Run the classification
                 */
                virtual Exception& run() {
                    if (!beforeClassification())
                        return exception;

                    benchmark.benchmark([this]() {
                        error = run_classifier(&signal, &result, _isDebugEnabled);
                    });

                    if (error != EI_IMPULSE_OK)
                        return exception.set("Classification error");

                    afterClassification();
                    breakTiming();

                    return exception.clear();
                }

            protected:
                bool _isDebugEnabled;

                /**
                 * Actions to perform before classification
                 */
                virtual bool beforeClassification() = 0;

                /**
                 * Actions to perform after classification
                 */
                virtual void afterClassification() = 0;

                /**
                 * Update timing information
                 */
                void breakTiming() {
                    timing.dsp = result.timing.dsp;
                    timing.classification = result.timing.classification;
                    timing.anomaly = result.timing.anomaly;
                    timing.total = timing.dsp + timing.classification + timing.anomaly;
                }
            };
        }
    }
}

#endif // ELOQUENT_ESP32CAM_EDGEIMPULSE_CLASSIFIER_H
