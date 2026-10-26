idf_component_register(SRCS "energy_meter.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash mqtt json)
