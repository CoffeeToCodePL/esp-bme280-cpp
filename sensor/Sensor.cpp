/*
 * Copyright (c) 2023 Michał Łubiński <mlubinski@coffeetocode.pl>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "Sensor.h"
#include "serial/I2CSerial.h"
#include "drivers/bme280/bme280.h"

#include <esp_log.h>

namespace {
  constexpr const char* TAG = "BME280";

  void delay_us(uint32_t period, void*)
  {
    vTaskDelay(period / portTICK_PERIOD_MS);
  }
}

namespace BME280 {
  template<class SerialType, class ConfigType>
  Sensor<SerialType, ConfigType>::Sensor()
    : interface()
    , device({
        .chip_id = BME280_CHIP_ID,
        .intf = interface.type(),
        .intf_ptr = interface.pointer(),
        .intf_rslt = BME280_OK,
        .read = &SerialType::read,
        .write = &SerialType::write,
        .delay_us = delay_us,
        .calib_data = {},
      })
  {
  }

  template<class SerialType, class ConfigType>
  bool Sensor<SerialType, ConfigType>::init(const ConfigType& config)
  {
    if (interface.init(config) != ESP_OK) {
      ESP_LOGE(TAG, "Could not initialize BME280 serial interface");
      return false;
    }

    if (bme280_init(&device) != BME280_OK) {
      ESP_LOGE(TAG, "Could not initialize BME280");
      return false;
    }

    const bme280_settings settings = {
      .osr_p = BME280_OVERSAMPLING_16X,
      .osr_t = BME280_OVERSAMPLING_2X,
      .osr_h = BME280_OVERSAMPLING_1X,
      .filter = BME280_FILTER_COEFF_16,
      .standby_time = BME280_STANDBY_TIME_1000_MS,
    };

    if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &device) != BME280_OK) {
      ESP_LOGE(TAG, "Could not set BME280 settings");
      return false;
    }

    if (bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &device) != BME280_OK) {
      ESP_LOGE(TAG, "Could not configure BME280 power mode");
      return false;
    }

    return true;
  }

  template<class SerialType, class ConfigType>
  void Sensor<SerialType, ConfigType>::reset()
  {
    bme280_soft_reset(&device);
    interface.reset();
  }

  template<class SerialType, class ConfigType>
  std::optional<bme280_data> Sensor<SerialType, ConfigType>::poll()
  {
    bme280_data sensorData{};

    if (bme280_get_sensor_data(BME280_ALL, &sensorData, &device) != BME280_OK) {
      ESP_LOGW(TAG, "Could not get sensor data from BME280");
      return std::nullopt;
    }

    return sensorData;
  }

  template class BME280::Sensor<BME280::I2C::Serial, BME280::I2C::Config>;
} // BME280