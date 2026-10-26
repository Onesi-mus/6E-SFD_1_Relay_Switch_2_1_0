idf_component_register(SRCS "mqtt_app_handler.c"
                       INCLUDE_DIRS "."
                       REQUIRES app_config relay_controls storage_handler mqtt ota energy_meter scheduler json switch_input mesh

)
