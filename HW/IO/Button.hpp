//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_BUTTON_HPP
#define TOMO_A4BOARD_BUTTON_HPP

struct Button{
    explicit Button(BTN_TYPE incomeType): type(incomeType){}
    const BTN_TYPE type;
    inline constexpr BTN_TYPE getType(){return type;}
};

#endif //TOMO_A4BOARD_BUTTON_HPP
