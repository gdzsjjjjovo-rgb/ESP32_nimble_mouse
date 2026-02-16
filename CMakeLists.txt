idf_component_register(
    SRCS "main.c" "mouse_report_stub.c" "esp_hid_gap.c" "print_report_map.c" "nimble.c" 
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_hid driver
)
