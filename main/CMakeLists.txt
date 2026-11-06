idf_component_register(SRCS "main.c" "link.c" "rf_capture.c" "spi_link.c" "stream.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_uart
                                     esp_hw_support esp_phy esp_rom esp_timer heap soc)
