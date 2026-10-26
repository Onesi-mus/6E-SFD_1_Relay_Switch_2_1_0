idf_component_register(SRCS "wifi_manager.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash app_config esp_wifi esp_timer)