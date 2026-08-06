idf_component_register(SRCS "main.c" "board.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash bt driver esp_timer)
