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

#include "I2CSerial.h"

#include <esp_log.h>

namespace BME280::I2C {
  Serial::Serial()
    : BME280::SerialBase<Serial, Config>()
  {
  }

  Serial::~Serial()
  {
    reset();
  }

  bme280_intf Serial::type()
  {
    return BME280_I2C_INTF;
  }

  bme280_intf_ptr_t Serial::pointer()
  {
    return static_cast<bme280_intf_ptr_t>(&interface);
  }

  esp_err_t Serial::init(const Config& config)
  {
    return interface.initialize(config.i2cPort,
                                config.sdaPin,
                                config.sclPin,
                                config.clockSpeed);
  }

  void Serial::reset()
  {
    interface.reset();
  }

  BME280_INTF_RET_TYPE Serial::read(uint8_t registerAddress,
                                    uint8_t* buffer,
                                    uint32_t bufferLength,
                                    void* interfacePtr)
  {
    auto interface = static_cast<I2CMaster*>(interfacePtr);

    if (!interface) {
      return BME280_E_NULL_PTR;
    }

    if (interface->readBytes(BME280_I2C_ADDR_PRIM,
                             registerAddress,
                             { buffer, bufferLength }) != ESP_OK) {
      return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
  }

  BME280_INTF_RET_TYPE Serial::write(uint8_t registerAddress,
                                     const uint8_t* data,
                                     uint32_t dataLength,
                                     void* interfacePtr)
  {
    auto interface = static_cast<I2CMaster*>(interfacePtr);

    if (!interface) {
      return BME280_E_NULL_PTR;
    }

    if (interface->writeBytes(BME280_I2C_ADDR_PRIM,
                              registerAddress,
                              { data, dataLength }) != ESP_OK) {
      return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
  }
}