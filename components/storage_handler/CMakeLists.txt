idf_component_register(
    SRCS "storage_handler.c"
    INCLUDE_DIRS "."
    REQUIRES app_config nvs_flash spiffs json relay_controls  
)