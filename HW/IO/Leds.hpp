//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_LEDS_HPP
#define TOMO_A4BOARD_LEDS_HPP

class Led{
public:
    Led() = delete;
    const Led& operator=(const Led &) = delete;
    Led& operator=(Led &) = delete;

    explicit constexpr Led(GPIO_TypeDef* incomePort, uint32_t incomePin):
        port(incomePort),
        pin(incomePin)
        {};

    inline void turnLedOn(){
        port->BSRR = (uint32_t)pin;
    }
    inline void turnLedOff(){
        port->BRR = (uint32_t)pin;
    }
    inline void toggleLed(){
        uint32_t odr = port->ODR;
        port->BSRR = ((odr & pin) << 16U) | (~odr & pin);
    }
private:
    GPIO_TypeDef* port;
    uint32_t pin;
};

#endif //TOMO_A4BOARD_LEDS_HPP
