//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_IOS_HPP
#define TOMO_A4BOARD_IOS_HPP

enum LOGIC_LEVEL{
    LOW = 0,
    HIGH = 1,
};
enum OUTPUT_TYPE{
    BUCKY_BRAKE,
    LASER_CENTERING,
    GRID_120,
    GRID_180,
    BUCKY_READY
};
enum INPUT_TYPE{
    GRID_CENTER,
    GRID_120_DETECT,
    GRID_180_DETECT,
    ON_TOMO,
    BUCKY_CALL
};
enum MOTOR_OUTS{
    CURRENT_WIND,
    STEP_PIN,
    DIR_PIN,
    ENABLE_PIN,
    RESET_PIN,
};

struct PinReadable
{
};

struct PinWriteable
{
};

template<typename DataLabel, typename Interface>
class IOS{
public:
    LOGIC_LEVEL currentState = LOW;
    const DataLabel dataType;

    template<typename T = Interface, class = typename std::enable_if_t<std::is_base_of<PinReadable, T>::value>>
    inline LOGIC_LEVEL getValue(){
        LOGIC_LEVEL retVal;
        if ((port->IDR & pin) != (uint32_t)LOGIC_LEVEL::LOW) retVal = LOGIC_LEVEL::HIGH;
        else retVal = LOGIC_LEVEL::LOW;
        if(inverted) return (retVal ? LOGIC_LEVEL::LOW : LOGIC_LEVEL::HIGH);
        else return retVal;
    }

    template<typename T = Interface, class = typename std::enable_if_t<std::is_base_of<PinReadable, T>::value>>
    inline LOGIC_LEVEL refresh(){
        currentState = getValue();
        return currentState;
    }

    template<typename T = Interface, class = typename std::enable_if_t<std::is_base_of<PinWriteable, T>::value>>
    inline void setValue(LOGIC_LEVEL value){
        if (value) port->BSRR = (uint32_t)pin;
        else port->BRR = (uint32_t)pin;
        currentState = value;
    }

    template<typename T = Interface, class = typename std::enable_if_t<std::is_base_of<PinWriteable, T>::value>>
    inline void toggleData(){
        uint32_t odr = port->ODR;
        port->BSRR = ((odr & pin) << 16U) | (~odr & pin);
    }

    IOS() = delete;
    const IOS& operator=(const IOS &) = delete;
    IOS& operator=(IOS &) = delete;

    void setInverted() {
        inverted = true;
    }

    constexpr explicit IOS(DataLabel incomeData, GPIO_TypeDef* incomePortPtr, uint16_t incomePin):
        dataType(incomeData),
        port(incomePortPtr),
        pin(incomePin)
    {};
protected:
private:
    GPIO_TypeDef* port;
    uint16_t pin;
    bool inverted = false;
};

#endif //TOMO_A4BOARD_IOS_HPP