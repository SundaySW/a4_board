#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "gpio.h"
#include "iwdg.h"

#include <list>
#include "../BoardApp/MainController.hpp"

using inIOS = IOS<INPUT_TYPE, PinReadable>;
using InList = std::list<inIOS>;
InList inputsList;
IOS gridCenter = inIOS(GRID_CENTER, GRID_END_POINT_GPIO_Port, GRID_END_POINT_Pin);
IOS grid120Detect = inIOS(GRID_CENTER, GRID_120_DETECT_GPIO_Port, GRID_120_DETECT_Pin);
IOS grid180Detect = inIOS(GRID_CENTER, GRID_180_DETECT_GPIO_Port, GRID_180_DETECT_Pin);
IOS onTomo = inIOS (ON_TOMO, ON_TOMO_GPIO_Port, ON_TOMO_Pin);
IOS buckyCall = inIOS (BUCKY_CALL, BUCKY_CALL_GPIO_Port, BUCKY_CALL_Pin);

using outIOS = IOS<OUTPUT_TYPE, PinWriteable>;
using OutList = std::list<outIOS>;
OutList outputsList;
IOS buckyBrake = outIOS(BUCKY_BRAKE, BUCKY_BRAKE_GPIO_Port, BUCKY_BRAKE_Pin);
IOS laserCentering = outIOS(LASER_CENTERING, LASER_CENTERING_GPIO_Port, LASER_CENTERING_Pin);
IOS grid120 = outIOS(GRID_120, GRID_120_GPIO_Port, GRID_120_Pin);
IOS grid180 = outIOS(GRID_180, GRID_180_GPIO_Port, GRID_180_Pin);
IOS buckyReady = outIOS(BUCKY_READY, BUCKY_READY_GPIO_Port, BUCKY_READY_Pin);

const Button<GRID_BUTTON> btn_grid;
const Button<PUSHBUTTON_BUCKYBRAKE> btn_bucky;

StepperCfg cfg;
auto motorController = MotorController();
auto mainController = MainController(inputsList, outputsList, motorController);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1) mainController.update();

    if(htim->Instance == TIM3) HAL_IWDG_Refresh(&hiwdg);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4) motorController.motor_refresh();
}

volatile bool btn_buckyBrake_pressed = false;
volatile bool btn_grid_pressed = false;
volatile uint32_t time_gridBTN_pressed;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
        case PUSHBUTTON_BUCKYBRAKE_Pin:
            if(btn_buckyBrake_pressed)
                mainController.btn_event(btn_bucky, LOW);
            else
                mainController.btn_event(btn_bucky, HIGH);
            btn_buckyBrake_pressed = !btn_buckyBrake_pressed;
            break;
        case GRID_BUTTON_Pin:
            if(btn_grid_pressed){
                if((HAL_GetTick() - time_gridBTN_pressed) > TIME_GRID_BTN_LONG_PRESS) {
                    btn_grid_pressed = false;
                    mainController.btn_event(btn_grid, HIGH);
                }
            }else{
                btn_grid_pressed = true;
                time_gridBTN_pressed = HAL_GetTick();
            }
            break;
        default:
            break;
    }
}

void EXTI_clear_enable(){
    __HAL_GPIO_EXTI_CLEAR_IT(PUSHBUTTON_BUCKYBRAKE_Pin);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    __HAL_GPIO_EXTI_CLEAR_IT(GRID_BUTTON_Pin);
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void SystemClock_Config();

StepperCfg& DIPSwitches_configureDriver();

int main()
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_FDCAN1_Init();
    MX_TIM1_Init();
    //MX_TIM2_Init();
    MX_TIM4_Init();
    MX_TIM3_Init();
    MX_IWDG_Init();

    inputsList.push_back(gridCenter);
    inputsList.push_back(grid120Detect);
    inputsList.push_back(grid180Detect);
    inputsList.push_back(onTomo);
    inputsList.push_back(buckyCall);
    outputsList.push_back(buckyBrake),
    outputsList.push_back(laserCentering),
    outputsList.push_back(grid120),
    outputsList.push_back(grid180),
    outputsList.push_back(buckyReady);

    motorController.load_driver(DIPSwitches_configureDriver());

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim3);

    EXTI_clear_enable();

    mainController.init_procedure();

    while (true){}
}

StepperCfg& DIPSwitches_configureDriver(){
    cfg.Vmin = START_SPEED;
    cfg.htim = &htim4;
    if(HAL_GPIO_ReadPin(CONFIG_1_GPIO_Port, CONFIG_1_Pin)){
        if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
            cfg.Vmax = CONFIG1_SPEED;
            cfg.A = CONFIG1_ACCELERATION;
        }else{
            cfg.Vmax = CONFIG2_SPEED;
            cfg.A = CONFIG2_ACCELERATION;
        }
    }else{
        if(HAL_GPIO_ReadPin(CONFIG_2_GPIO_Port, CONFIG_2_Pin)){
            cfg.Vmax = CONFIG3_SPEED;
            cfg.A = CONFIG3_ACCELERATION;
        }else{
            cfg.Vmax = CONFIG4_SPEED;
            cfg.A = CONFIG4_ACCELERATION;
        }
    }
    //TODO реализовать 3 dip на инверсию движения
    //if(HAL_GPIO_ReadPin(CONFIG_3_GPIO_Port, CONFIG_3_Pin)) DIRECTION::BACKWARDS = 1;
    //else motor_instance_1.DIR_pin_logic_level_inverted = 0;
   return cfg;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config()
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the peripherals clocks
    */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1){}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */