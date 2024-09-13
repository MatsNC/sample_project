idf_component_register(SRCS
    "main.c"
    "nvs/nvs.c"
    "timer/timer.c"
    "wifi/wifi.c"
    "i2c/i2c.c"
    INCLUDE_DIRS "."
    "nvs"
    "timer"
    "wifi"
    "i2c")
