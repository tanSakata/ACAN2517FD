#include "RasPiHardwareSPI.h"
#include <stdexcept>

namespace acan2517fd {

ROS2HardwareSPI::ROS2HardwareSPI(uint8_t spi_channel, uint32_t spi_speed, uint8_t cs_pin)
    : spi_handle_(-1), cs_pin_(cs_pin), spi_speed_(spi_speed), configuration_mode_(true) {
    // GPIO初期化
    if (gpioInitialise() < 0) {
        throw std::runtime_error("Failed to initialize GPIO");
    }

    // SPIチャネルを開く
    spi_handle_ = spiOpen(spi_channel, spi_speed_, 0); // mode 0
    if (spi_handle_ < 0) {
        throw std::runtime_error("Failed to open SPI channel");
    }

    // CSピンを出力に設定
    gpioSetMode(cs_pin_, PI_OUTPUT);
    deassertCS();
}

void ROS2HardwareSPI::beginTransaction(bool configuration_mode) {
    if (configuration_mode_ != configuration_mode) {
        if (configuration_mode) {
            spi_speed_ = 1000UL * 1000; // 設定モード用速度
        }
        spiHandleSetBitsPerWord(spi_handle_, 8);
        configuration_mode_ = configuration_mode;
    }
    assertCS();
}

void ROS2HardwareSPI::endTransaction() {
    deassertCS();
}

int ROS2HardwareSPI::transfer(const uint8_t *buffer, int length) {
    std::vector<uint8_t> rx_buffer(length);
    int result = spiXfer(spi_handle_, const_cast<char*>(reinterpret_cast<const char*>(buffer)), 
                         reinterpret_cast<char*>(rx_buffer.data()), length);
    if (result < 0) {
        throw std::runtime_error("SPI transfer failed");
    }
    return result;
}

int ROS2HardwareSPI::transfer16(const uint16_t data) {
    uint16_t rx_data = 0;
    int result = spiXfer(spi_handle_, reinterpret_cast<char*>(const_cast<uint16_t*>(&data)), 
                         reinterpret_cast<char*>(&rx_data), sizeof(data));
    if (result < 0) {
        throw std::runtime_error("SPI 16-bit transfer failed");
    }
    return result;
}

void ROS2HardwareSPI::initCS() {
    gpioSetMode(cs_pin_, PI_OUTPUT);
    deassertCS();
}

inline void ROS2HardwareSPI::assertCS() {
    gpioWrite(cs_pin_, 0);
}

inline void ROS2HardwareSPI::deassertCS() {
    gpioWrite(cs_pin_, 1);
}

void ROS2HardwareSPI::setSPIClock(const uint32_t spiClock) {
    spi_speed_ = spiClock;
}

ROS2HardwareSPI::~ROS2HardwareSPI() {
    if (spi_handle_ >= 0) {
        spiClose(spi_handle_);
    }
    gpioTerminate();
}

}
