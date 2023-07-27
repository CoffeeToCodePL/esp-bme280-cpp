idf_component_register(
        SRC_DIRS "serial" "sensor" "drivers/bme280"
        INCLUDE_DIRS "./"
        REQUIRES esp-i2c-cpp
)
