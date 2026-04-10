idf_component_register(SRCS "AIS.cpp" "ESP32_SIM7670X_AIS.cpp" "nmea_parser.c"
                    REQUIRES i2c_bus esp-tls lwip esp_timer esp_idf_lib_helpers esp_driver_tsens esp_driver_uart esp_http_client app_update esp_driver_gpio esp_psram
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES server_root_cert.pem)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Wno-missing-field-initializers -Wno-write-strings)
