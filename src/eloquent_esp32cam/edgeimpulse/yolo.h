#ifndef ELOQUENT_ESP32CAM_EDGEIMPULSE_yolo_H
#define ELOQUENT_ESP32CAM_EDGEIMPULSE_yolo_H

#include <string> // Include the C++ standard library for std::string

#if defined(EI_CLASSIFIER_OBJECT_DETECTION)
    #include "./image.h"
    #include "./bbox.h"
    #include "./yolo_daemon.h"
    
    using eloq::ei::bbox_t;

    namespace Eloquent {
        namespace Esp32cam {
            namespace EdgeImpulse {
                /**
                 * Run Edge Impulse yolo model
                 */
                class yolo : public ImageClassifier {
                public:
                    bbox_t first;
                    yoloDaemon<yolo> daemon;

                    /**
                     *
                     */
                    yolo() :
                        ImageClassifier(),
                        first("", 0, 0, 0, 0, 0),
                        daemon(this) {
                    }

                    /**
                     * Check if objects were found
                     */
                    bool found() {
                        return result.bounding_boxes[0].value != 0;
                    }

                    /**
                     * Check if objects were found
                     */
                    bool foundAnyObject() {
                        return found();
                    }

                    /**
                     * Run function on each bounding box found
                     */
                    template<typename Callback>
                    void forEach(Callback callback) {
                        for (size_t ix = 0, i = 0; ix < result.bounding_boxes_count; ix++) {
                            auto bb = result.bounding_boxes[ix];
                            bbox_t bbox(
                                bb.label,
                                bb.value,
                                bb.x,
                                bb.y,
                                bb.width,
                                bb.height
                            );

                            if (bbox.proba > 0)
                                callback(i++, bbox);
                        }
                    }

                    /**
                     *
                     */
                    ei_impulse_result_bounding_box_t at(size_t ix) {
                        return result.bounding_boxes[ix];
                    }

                    /**
                     * Get count of (non background) bounding boxes
                     */
                    size_t count() {
                        size_t count = 0;

                        for (size_t ix = 0, i = 0; ix < result.bounding_boxes_count; ix++) {
                            auto bb = result.bounding_boxes[ix];

                            if (bb.value != 0)
                                count++;
                        }

                        return count;
                    }

                    /**
                     * Convert to JSON string
                     */
                    std::string toJSON() {
                        std::string json = "[]";

                        if (!found())
                            return json;

                        json = "[";

                        forEach([&json](uint8_t i, bbox_t bbox) {
                            if (i > 0)
                                json += ',';

                            json += '{';
                            json += "\"label\":\"";
                            json += bbox.label;
                            json += "\",\"proba\":";
                            json += std::to_string(bbox.proba);
                            json += ",\"x\":";
                            json += std::to_string(bbox.x);
                            json += ",\"y\":";
                            json += std::to_string(bbox.y);
                            json += ",\"w\":";
                            json += std::to_string(bbox.width);
                            json += ",\"h\":";
                            json += std::to_string(bbox.height);
                            json += '}';
                        });

                        json += ']';
                        
                        return json;
                    }

                    /**
                     * Test if a MQTT payload is available
                     */
                    bool shouldPub() {
                        return found();
                    }

                protected:

                    /**
                     * Run actions after classification results
                     */
                    virtual void afterClassification() {
                        if (found()) {
                            auto bb = result.bounding_boxes[0];
                            first.label = bb.label;
                            first.proba = bb.value;
                            first.setDimensions(bb.x, bb.y, bb.width, bb.height);
                        }
                    }
                };
            }
        }
    }

    namespace eloq {
        namespace ei {
            static Eloquent::Esp32cam::EdgeImpulse::yolo yolo;
        }
    }

#else
#error "EdgeImpulse yolo library not found"
#endif

#endif //ELOQUENT_ESP32CAM_EDGEIMPULSE_yolo
