//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_BUTTON_HPP
#define TOMO_A4BOARD_BUTTON_HPP


template<BTN_TYPE incomeType>
struct Button{
    static const BTN_TYPE type = incomeType;
    static inline constexpr BTN_TYPE getType(){
        return type;
    }
};

#endif //TOMO_A4BOARD_BUTTON_HPP
