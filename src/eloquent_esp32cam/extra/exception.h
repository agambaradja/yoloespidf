#ifndef ELOQUENT_EXCEPTION_H
#define ELOQUENT_EXCEPTION_H

#include <string>
#include <esp_log.h>

namespace Eloquent {
    namespace Error {
        /**
         * Application exception
         */
        class Exception {
        public:
            /**
             * Constructor
             */
            Exception(const char* tag) : 
                _tag(tag), 
                _isSevere(true), // Initialize _isSevere before _message
                _message("") {
            }

            /**
             * Test if there's an exception
             */
            operator bool() const {
                return !isOk();
            }

            /**
             * Test if there's an exception
             */
            bool isOk() const {
                return _message.empty();
            }

            /**
             * Test if exception is severe
             */
            bool isSevere() const {
                return _isSevere && !isOk();
            }

            /**
             * Mark error as not severe
             */
            Exception& soft() {
                _isSevere = false;
                return *this;
            }

            /**
             * Set exception message
             */
            Exception& set(const std::string& error) {
                _message = error;
                _isSevere = true;

                if (!error.empty()) {
                    const char *c_str = error.c_str();

                    if (_isSevere)
                        ESP_LOGE(_tag, "%s", c_str);
                    else
                        ESP_LOGW(_tag, "%s", c_str);
                }

                return *this;
            }

            /**
             * Clear exception
             */
            Exception& clear() {
                return set("");
            }

            /**
             * Propagate exception from another object
             */
            template<typename Other>
            Exception& propagate(Other& other) {
                set(other.exception.toString());
                return *this;
            }

            /**
             * Convert exception to string
             */
            inline std::string toString() const {
                return _message;
            }

            /**
             * Convert exception to const char*
             */
            inline const char* toCString() const {
                return _message.c_str();
            }

            /**
             * Static method to return no exception
             */
            static Exception none() {
                return Exception("");
            }

        protected:
            const char* _tag;
            bool _isSevere;
            std::string _message;
        };
    }
}

#endif
