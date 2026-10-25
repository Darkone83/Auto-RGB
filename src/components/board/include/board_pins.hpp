#pragma once

#include <array>
#include <cstddef>
#include "driver/gpio.h"

namespace board {

// Matches the current Rev-A schematic: CH1..CH8 = GPIO1..GPIO8.
// GPIO3 is an ESP32-S3 strapping pin; if CH3 moves in a later board revision,
// this is the only mapping that needs to change.
constexpr gpio_num_t kStatusLedGpio = GPIO_NUM_21; // Waveshare ESP32-S3-Zero onboard WS2812

// Future Remote Control daughterboard on the 7-pin EXT header.
// EXT: 3V3, SDA, ENC_A, ENC_SW, ENC_B, SCL, GND
constexpr gpio_num_t kRemoteSdaGpio = GPIO_NUM_9;
constexpr gpio_num_t kRemoteSclGpio = GPIO_NUM_10;
constexpr gpio_num_t kRemoteEncoderAGpio = GPIO_NUM_11;
constexpr gpio_num_t kRemoteEncoderBGpio = GPIO_NUM_12;
constexpr gpio_num_t kRemoteEncoderSwitchGpio = GPIO_NUM_13;

constexpr std::array<gpio_num_t, 8> kChannelGpios = {
    GPIO_NUM_1,
    GPIO_NUM_2,
    GPIO_NUM_3,
    GPIO_NUM_4,
    GPIO_NUM_5,
    GPIO_NUM_6,
    GPIO_NUM_7,
    GPIO_NUM_8,
};

gpio_num_t channel_gpio(std::size_t channel);

} // namespace board
