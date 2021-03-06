//
// Created by 79162 on 25.09.2021.
//

#ifndef TOMO_A4BOARD_STEPPERMOTOR_HPP
#define TOMO_A4BOARD_STEPPERMOTOR_HPP

#include <cstdlib>
#include "../IO/IOS.hpp"

enum MOTOR_EVENT {
    EVENT_NULL = 0,
    EVENT_STOP, // 	move stopped
    EVENT_CSS,  //	constant speed reached
    EVENT_CSE   //  constant speed end
};

struct StepperCfg
{
    float A = CONFIG1_ACCELERATION;				// Acceleration in steps/second^2
    float Vmin = START_SPEED;					// Minimum speed in steps/second
    float Vmax = CONFIG1_SPEED;					// Maximum speed in steps/second
    const int criticalNofSteps = CRITICAL_N_OF_STEPS;
    TIM_HandleTypeDef *htim = &htim4;
};

class MotorController{
    using MOTOR_IOS = IOS<MOTOR_OUTS, PinWriteable>;
public:
    enum MODE
    {
        IDLE,
        ACCEL,
        CONST,
        DECCEL,
        in_ERROR
    };
    enum DIRECTION{
        BACKWARDS = 0,
        FORWARD = 1
    };

    MotorController() = default;

    void load_driver(StepperCfg &&cfg){
        A = cfg.A;
        Vmin = cfg.Vmin;
        Vmax = cfg.Vmax;
        criticalNofSteps = cfg.criticalNofSteps;
        htim = cfg.htim;
    }

    //TODO вызвать в колбэке таймера импульсов мотора
    bool noReturn = false;
    void motor_refresh(){
        if(mode == in_ERROR) return;
        if(accelerationMode) reCalcSpeed();
        else currentStep++;
        if(currentStep >= criticalNofSteps){
            if(noReturn) stopMotor();
            else changeDirection();
        }
        if(direction_changed > 1){
            stopMotor();
//            mode = in_ERROR;
        }
        regValueCalc();
    }

    void get_open_position(){
        accelerationMode = false;
        V = LOAD_UNLOAD_SPEED;
        setDirection(FORWARD);
        noReturn = true;
        startMotor();
    }

    void get_center_position(bool initMove){
        accelerationMode = false;
        V = LOAD_UNLOAD_SPEED;
        if(initMove) setDirection(FORWARD);
        else setDirection(BACKWARDS);
        startMotor();
    }

    void exposition(){
        accelerationMode = true;
        V = Vmin;
        setDirection(FORWARD);
        startMotor();
    }

    inline void stopMotor(){
        if(motorMoving){
            HAL_TIM_PWM_Stop_IT(htim, TIM_CHANNEL_2);
            enable.setValue(HIGH);
            motorMoving = false;
        }
        noReturn = false;
    }

    inline void changeDirection(){
        if(!accelerationMode) direction_changed += 1;
        currentDirection = currentDirection ? BACKWARDS : FORWARD;
        direction.setValue(LOGIC_LEVEL(currentDirection));
        currentStep = 0;
        accel_step = 0;
        mode = MODE::ACCEL;
    }

    constexpr bool isMotorMoving() const {
        return motorMoving;
    }

    MODE getMode() const {
        return mode;
    }

    MOTOR_EVENT getEvent() const {
        return event;
    }

    DIRECTION getCurrentDirection() const {
        return currentDirection;
    }

private:
    MOTOR_IOS step = MOTOR_IOS(STEP_PIN, STEP_GPIO_Port, STEP_Pin);
    MOTOR_IOS direction = MOTOR_IOS(DIR_PIN, DIR_GPIO_Port, DIR_Pin);
    MOTOR_IOS enable = MOTOR_IOS(ENABLE_PIN, ENABLE_GPIO_Port, ENABLE_Pin);
    MOTOR_IOS current = MOTOR_IOS(CURRENT_WIND, CURRENT_WIND_GPIO_Port, CURRENT_WIND_Pin);
    TIM_HandleTypeDef *htim;

    int currentStep = 0;
    int accel_step = 0;

    float A = 40.0f;				            // step^2 / second
    float V = 0.0f;
    float Vmin = START_SPEED;                   // Minimum speed in steps/second
    float Vmax = CONFIG1_SPEED;                 // Maximum speed in steps/second
    int criticalNofSteps = CRITICAL_N_OF_STEPS;

    DIRECTION currentDirection = FORWARD;
    MODE mode = IDLE;
    MOTOR_EVENT event = EVENT_STOP;
    bool motorMoving = false;
    bool accelerationMode = false;
    uint8_t direction_changed = 0;

    void startMotor(){
        if(!motorMoving){
            accel_step = 0;
            currentStep = 0;
            mode = MODE::ACCEL;
            direction_changed = 0;
            motorMoving = true;
            enable.setValue(LOW);
            regValueCalc();
            HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_2);
        }
    }
    inline void regValueCalc(){
        if(V > 0){
            int buf = (int) (10000000 / V);
            if(buf > 0 && buf < 65535){
                __HAL_TIM_SET_AUTORELOAD(htim, buf);
                __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,buf/2);
            }
        }
    }

    void setDirection(DIRECTION newDirection){
        currentDirection = newDirection;
        direction.setValue(LOGIC_LEVEL(newDirection));
    }

    inline void reCalcSpeed(){
        event = EVENT_NULL;
        if (mode == IDLE) return;
        switch (mode)
        {
            case MODE::ACCEL:
            {
                if (V >= Vmax)
                {
                    V = Vmax;
                    event = EVENT_CSS;
                }else
                    V += A;
                if (accel_step >= criticalNofSteps / 3)
                {
                    mode = MODE::DECCEL;
                    break;
                }
//                V += A / abs(V);
                accel_step++;
            }
            break;

            case MODE::CONST:
            {
                if (currentStep + accel_step >= criticalNofSteps) {
                    mode = MODE::DECCEL;
                    event = EVENT_CSS;
                }
            }
            break;

            case MODE::DECCEL:
            {
                if (accel_step <= 0)
                {
//                    stopMotor();
//                    mode = MODE::IDLE;
//                    event = EVENT_STOP;
                    changeDirection();
                    break;
                }else{
//                    V -= A/abs(V);
                    V -= A;
                    if (V < Vmin) V = Vmin;
                    accel_step--;
                }
            }
            break;

            default:
                break;
        }

        if (mode == ACCEL || mode == CONST || mode == DECCEL)
            currentStep++;
    }
};

#endif //TOMO_A4BOARD_STEPPERMOTOR_HPP