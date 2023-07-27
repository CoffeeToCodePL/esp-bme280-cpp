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

#ifndef ESP_BME280_CPP_SERIALBASE_H
#define ESP_BME280_CPP_SERIALBASE_H

#include "drivers/bme280/bme280_defs.h"

#include <esp_err.h>
#include <span>

namespace BME280 {
  using bme280_intf_ptr_t = void*;

  template<typename SerialType, typename ConfigType>
  class SerialBase
  {
  public:
    SerialBase() = default;
    virtual ~SerialBase() = default;

    [[nodiscard]] bme280_intf type();
    [[nodiscard]] bme280_intf_ptr_t pointer();

    esp_err_t init(const ConfigType& config);
    void reset();

    static BME280_INTF_RET_TYPE read(uint8_t registerAddress,
                                     uint8_t* registerData,
                                     uint32_t dataLength,
                                     void* interfacePtr);
    static BME280_INTF_RET_TYPE write(uint8_t registerAddress,
                                      const uint8_t* registerData,
                                      uint32_t dataLength,
                                      void* interfacePtr);
  };

  template<typename SerialType, typename ConfigType>
  bme280_intf SerialBase<SerialType, ConfigType>::type()
  {
    return static_cast<SerialType*>(this)->type();
  }

  template<typename SerialType, typename ConfigType>
  bme280_intf_ptr_t SerialBase<SerialType, ConfigType>::pointer()
  {
    return static_cast<SerialType*>(this)->pointer();
  }

  template<typename SerialType, typename ConfigType>
  esp_err_t SerialBase<SerialType, ConfigType>::init(const ConfigType& config)
  {
    return static_cast<SerialType*>(this)->init(config);
  }

  template<typename SerialType, typename ConfigType>
  void SerialBase<SerialType, ConfigType>::reset()
  {
    static_cast<SerialType*>(this)->reset();
  }

  template<typename SerialType, typename ConfigType>
  BME280_INTF_RET_TYPE SerialBase<SerialType, ConfigType>::read(uint8_t registerAddress,
                                                                uint8_t* registerData,
                                                                uint32_t dataLength,
                                                                void* interfacePtr)
  {
    return SerialType::read(registerAddress, registerData, dataLength, interfacePtr);
  }

  template<typename SerialType, typename ConfigType>
  BME280_INTF_RET_TYPE SerialBase<SerialType, ConfigType>::write(uint8_t registerAddress,
                                                                 const uint8_t* registerData,
                                                                 uint32_t dataLength,
                                                                 void* interfacePtr)
  {
    return SerialType::write(registerAddress, registerData, dataLength, interfacePtr);
  }
}

#endif // ESP_BME280_CPP_SERIALBASE_H
