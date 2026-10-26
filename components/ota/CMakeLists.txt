idf_component_register(SRCS "ota.c"
                       INCLUDE_DIRS "."
                       REQUIRES mesh esp_wifi mqtt esp_https_ota app_update esp_http_client esp-tls json app_config nvs_flash

)