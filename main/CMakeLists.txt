
idf_component_register(
    SRCS "esp32_Mavlink.c"
    INCLUDE_DIRS "."
    REQUIRES mavlink nvs_flash esp_driver_uart
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=address-of-packed-member
)
