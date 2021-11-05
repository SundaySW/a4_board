//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_MAINCONTROLLER_HPP
#define TOMO_A4BOARD_MAINCONTROLLER_HPP

#include "../HW/StepperMotor/StepperMotor.hpp"
#include "../HW/IO/IOS.hpp"
#include "../HW/IO/Button.hpp"
#include <list>

class MainController {
    using inData = IOS<INPUT_TYPE, PinReadable>;
    using InList = std::list<inData>;
    using outData = IOS<OUTPUT_TYPE, PinWriteable>;
    using OutList = std::list<outData>;
public:
    const MainController& operator=(const MainController &) = delete;
    MainController& operator=(MainController &) = delete;
    MainController() = default;

//   explicit MainController(InList &incomeInputDataList, OutList &incomeOutputDataList, MotorController incomeMotorController):
//        inputDataList{incomeInputDataList},
//        outputDataList{incomeOutputDataList},
//        motorController(incomeMotorController)
//    {
//        for(auto &data: inputDataList) {
//            if(data.dataType == GRID_CENTER)        gridCenter = &data.currentState;
//            if(data.dataType == ON_TOMO)            onTomo = &data.currentState;
//            if(data.dataType == BUCKY_CALL)         buckyCall = &data.currentState;
//            if(data.dataType == GRID_120_DETECT)    grid120Detect = &data.currentState;
//            if(data.dataType == GRID_180_DETECT)    grid180Detect = &data.currentState;
//        }
//    }

    void init(InList incomeInputDataList, OutList incomeOutputDataList, MotorController &incomeMotorController){
       inputDataList = std::move(incomeInputDataList);
        for(auto &data: inputDataList) {
            if(data.dataType == GRID_CENTER)        gridCenter = &data.currentState;
            if(data.dataType == ON_TOMO)            onTomo = &data.currentState;
            if(data.dataType == BUCKY_CALL)         buckyCall = &data.currentState;
            if(data.dataType == GRID_120_DETECT)    grid120Detect = &data.currentState;
            if(data.dataType == GRID_180_DETECT)    grid180Detect = &data.currentState;
        }
       outputDataList = std::move(incomeOutputDataList);
       motorController = &incomeMotorController;
   }


    //TODO call таймере
    void update(){
        for(inData &data: inputDataList) data.refresh();

        if(motorController->getMode() == MotorController::in_ERROR)
            currentError = LIMIT_SWITCH_ERROR;

//        else if(*buckyCall || currentStatus == DEVICE_SCANING_TOMO_OFF || currentStatus == DEVICE_SCANING_TOMO_ON)
//            exposition_procedure();

        if(*gridCenter && !*buckyCall){
            motorController->stopMotor();
            currentStatus = DEVICE_STANDBY;
        }
        ////////
        if(currentStatus == DEVICE_STANDBY){
            for(outData &data : outputDataList){
                if(data.dataType == GRID_120) data.toggleData();
            }
        }
//        if(motorController->isMotorMoving()){
//            for(outData &data : outputDataList){
//                if(data.dataType == GRID_180) data.toggleData();
//            }
//        }
        ////////
        //if(currentError != NO_ERROR) errorHandler(currentError);
    }

    void btn_event(Button btn, LOGIC_LEVEL value){
       switch (btn.type) {
            case PUSHBUTTON_BUCKYBRAKE:
                if(currentStatus == DEVICE_STANDBY || currentStatus == DEVICE_BUCKYBRAKE) laser_centering_procedure(value);
                break;
            case GRID_BUTTON:
                if(currentStatus == DEVICE_STANDBY && rasterLoaded) raster_unload_procedure();
                else if(currentStatus == DEVICE_GRID_SUPPLY && !rasterLoaded) raster_load_procedure();
                break;
            default:
                break;
        }
    }

    void raster_load_procedure(){
        motorController->get_center_position();
        while (motorController->isMotorMoving());
        if((*grid120Detect && *grid180Detect) || (!*grid120Detect && !*grid180Detect)) currentError = GRID_TYPE_ERROR;
        else{
            rasterLoaded = true;
            for(outData &data : outputDataList){
                if(*grid120Detect) if(data.dataType == GRID_120) data.setValue(HIGH);
                if(*grid180Detect) if(data.dataType == GRID_180) data.setValue(HIGH);
            }
            currentStatus = DEVICE_STANDBY;
        }
    }

    void raster_unload_procedure(){
        currentStatus = DEVICE_GRID_SUPPLY;
        rasterLoaded = false;
        motorController->get_open_position();
        while (motorController->isMotorMoving());
        for(outData &data : outputDataList){
            if(data.dataType == GRID_120) data.setValue(LOW);
            if(data.dataType == GRID_180) data.setValue(LOW);
        }
    }

    void laser_centering_procedure(LOGIC_LEVEL value){
        for(auto &data: outputDataList){
            if(data.dataType == BUCKY_BRAKE || data.dataType == LASER_CENTERING) data.setValue(value);
        }
        currentStatus = value ?  DEVICE_BUCKYBRAKE : DEVICE_STANDBY;
    }

    void init_procedure(){
        currentStatus = DEVICE_INITIAL_MOVEMENT;
        motorController->get_center_position();
        while (motorController->isMotorMoving());
        currentStatus = DEVICE_STANDBY;
    }

    inline void exposition_procedure(){
        switch (currentStatus) {
            case DEVICE_STANDBY:
                if(*buckyCall && *onTomo) currentStatus = DEVICE_SCANING_TOMO_ON;
                else if(*buckyCall && !*onTomo) currentStatus = DEVICE_SCANING_TOMO_OFF;
                motorController->exposition();
              break;
            case DEVICE_SCANING_TOMO_OFF:
                if(motorController->getEvent() == EVENT_CSS){
                    for(auto &data: outputDataList) if(data.dataType == BUCKY_READY) data.setValue(HIGH);
                }
                if(!*buckyCall) {
                    currentStatus = DEVICE_INITIAL_MOVEMENT;
                    for(auto &data: outputDataList) if(data.dataType == BUCKY_READY) data.setValue(LOW);
                    init_procedure();
                }
              break;
            case DEVICE_SCANING_TOMO_ON:
                if(!*onTomo && !tomoSignal){
                    for(auto &data: outputDataList) if(data.dataType == BUCKY_READY) data.setValue(HIGH);
                    tomoSignal = true;
                }
                else if(*onTomo && tomoSignal){
                    tomoSignal = false;
                    for(auto &data: outputDataList) if(data.dataType == BUCKY_READY) data.setValue(LOW);
                }
                if(!*buckyCall) {
                    currentStatus = DEVICE_INITIAL_MOVEMENT;
                    for(auto &data: outputDataList) if(data.dataType == BUCKY_READY) data.setValue(LOW);
                    init_procedure();
                }
              break;
            default:
              break;
        }
    }

private:
    InList inputDataList;
    LOGIC_LEVEL *gridCenter;
    LOGIC_LEVEL *onTomo;
    LOGIC_LEVEL *buckyCall;
    LOGIC_LEVEL *grid120Detect;
    LOGIC_LEVEL *grid180Detect;

    OutList outputDataList;

    MotorController *motorController;

    BOARD_ERROR currentError = NO_ERROR;
    volatile DEVICE_STATUS currentStatus = DEVICE_STANDBY;
    bool rasterLoaded = true;
    bool tomoSignal = false;

    [[noreturn]] void errorHandler(BOARD_ERROR error){
        motorController->stopMotor();
        currentStatus = DEVICE_ERROR;
        switch (error) {
            case GRID_TYPE_ERROR:
                break;
            case LIMIT_SWITCH_ERROR:
                break;
            case STANDBY_MOVEMENT_ERROR:
                break;
            case ON_TOMO_BUCKY_CALL_ERROR:
                break;
            default:
                break;
        }
        Error_Handler();
        while (true){};
    }
};

#endif //TOMO_A4BOARD_MAINCONTROLLER_HPP
