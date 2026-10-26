idf_component_register(SRCS "aws_mqtt.c"
                       INCLUDE_DIRS "."
                       REQUIRES relay_controls esp_wifi mqtt mqtt_app_handler
                       EMBED_FILES
                                   "ca.pem"
)
