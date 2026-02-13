idf_component_register(
    SRCS "main.c" "mouse_report_stub.c" "esp_hid_gap.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash bt esp_hid driver
)
