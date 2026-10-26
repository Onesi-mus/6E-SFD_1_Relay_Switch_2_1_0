idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS ""
                       REQUIRES nvs_flash
                                app_update
                                app_config
                                wifi_manager
                                storage_handler
                                relay_controls
                                aws_mqtt
                                mqtt_app_handler
                                ota
                                energy_meter
                                scheduler
                                ble_prov
                                switch_input
                                led_indicator
                                esp_timer
                                esp_wifi
                                mesh
                                 )
