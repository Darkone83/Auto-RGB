#include "board_pins.hpp"

namespace board {

gpio_num_t channel_gpio(std::size_t channel)
{
    return channel < kChannelGpios.size() ? kChannelGpios[channel] : GPIO_NUM_NC;
}

} // namespace board
