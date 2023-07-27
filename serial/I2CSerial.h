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

#ifndef ESP_BME280_CPP_I2C_SERIAL_H
#define ESP_BME280_CPP_I2C_SERIAL_H

#include "serial/SerialBase.h"

#include <i2c.h>

namespace BME280::I2C {
  using I2CMaster = ::I2C::Master;

  /**
   * @brief Default configuration options.
   *
   * Values are:
   * - Port -> I2C_NUM_0
   * - SdaPin -> GPIO_NUM_21
   * - SclPin -> GPIO_NUM_22
   * - ClockSpeed -> 100000 (100 kHz)
   * - Timeout -> 1000 (ms)
   */
  namespace Defaults {
    static constexpr i2c_port_t Port = I2C_NUM_0;
    static constexpr gpio_num_t SdaPin = GPIO_NUM_21;
    static constexpr gpio_num_t SclPin = GPIO_NUM_22;
    static constexpr uint32_t ClockSpeed = 100000;
    static constexpr uint32_t Timeout = 1000;
  }

  /**
   * @brief Provides I2C configuration used initialize the driver.
   *
   * Defaults are:
   * - i2cPort set to I2C::Defaults::Port
   * - sdaPin set to I2C::Defaults::SdaPin
   * - sclPin set to I2C::Defaults::SclPin
   * - clockSpeed set to I2C::Defaults::ClockSpeed
   *
   * @see I2C::Defaults
   */
  struct Config
  {
    i2c_port_t i2cPort = Defaults::Port;
    gpio_num_t sdaPin = Defaults::SdaPin;
    gpio_num_t sclPin = Defaults::SclPin;
    uint32_t clockSpeed = Defaults::ClockSpeed;
  };

  class Serial : public BME280::SerialBase<Serial, Config>
  {
  public:
    Serial();
    ~Serial() override;

    [[nodiscard]] bme280_intf type();
    [[nodiscard]] bme280_intf_ptr_t pointer();

    esp_err_t init(const Config& config);
    void reset();

    static BME280_INTF_RET_TYPE read(uint8_t registerAddress,
                                     uint8_t* registerData,
                                     uint32_t dataLength,
                                     void* interfacePtr);
    static BME280_INTF_RET_TYPE write(uint8_t registerAddress,
                                      const uint8_t* registerData,
                                      uint32_t dataLength,
                                      void* interfacePtr);

  private:
    I2CMaster interface;
  };
}

#endif // ESP_BME280_CPP_I2C_SERIAL_H
