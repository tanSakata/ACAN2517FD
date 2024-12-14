#ifndef _ROS2_HARDWARE_SPI_H_
#define _ROS2_HARDWARE_SPI_H_

#include <pigpio.h>
#include <vector>
#include "SPIHardwareInterface.h"

namespace acan2517fd {

class ROS2HardwareSPI : public SPIHardwareInterface {
public:
    ROS2HardwareSPI(uint8_t spi_channel, uint32_t spi_speed, uint8_t cs_pin);

    virtual void beginTransaction(bool configuration_mode = false) override;

    virtual void endTransaction() override;

    virtual int transfer(const uint8_t *buffer, int length) override;

    virtual int transfer16(const uint16_t data) override;

    virtual void initCS() override;

    virtual inline void assertCS() override;

    virtual inline void deassertCS() override;

    void setSPIClock(const uint32_t spiClock) override;

    ~ROS2HardwareSPI();

private:
    int spi_handle_;       // SPIハンドル
    uint8_t cs_pin_;       // チップセレクトピン番号
    uint32_t spi_speed_;   // SPI通信速度
    bool configuration_mode_;
};

}

#endif
