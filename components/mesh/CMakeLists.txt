idf_component_register(SRCS "mesh.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_netif esp_event esp_http_server esp_hw_support wifi_manager)
