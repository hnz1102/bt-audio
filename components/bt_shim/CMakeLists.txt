idf_component_register(SRCS "bt_shim.c"
                       INCLUDE_DIRS "."
                       REQUIRES bt driver esp_adc freertos)
