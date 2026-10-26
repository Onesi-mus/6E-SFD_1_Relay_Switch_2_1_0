idf_component_register(
    SRCS "scheduler.c"
    INCLUDE_DIRS "."
    REQUIRES mqtt_app_handler json
)
