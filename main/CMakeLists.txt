idf_component_register(SRCS
    "main.c"
    "nvs/nvs.c"
    "timer/timer.c"
    "wifi/wifi.c"
    "i2c/i2c_master.c"
    "i2c/i2c_slave.c"
    "webserver/webserver.c"
    INCLUDE_DIRS "."
    "nvs"
    "timer"
    "wifi"
    "i2c"
    "webserver")
