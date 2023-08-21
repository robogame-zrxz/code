/**
 * @file motor.hpp
 * @author lavinal austin712@mail.ustc.edu.cn
 * @brief motor
 * @version 0.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "pid.hpp"
#include <stdint.h>

/* Exported macros -----------------------------------------------------------*/
const double PI = 3.1415926;
const double MOTOR_FULL_OMEGA = (230.0 / 60.0 * 2.0 * PI);
const double MOTOR_ENCODER_NUM_PER_RAD = (17.0 * 27.0 / 2.0 / PI);
const int32_t MOTOR_CALCULATE_PRESCALER = 32767;
const double MOTOR_CALCULATE_PERIOD = 0.05;

/* Exported types ------------------------------------------------------------*/
enum Control_Method
{
    Control_Method_OPENLOOP = 0,
    Control_Method_OMEGA,
    Control_Method_ANGLE
};

enum Rotate_Direction
{
    CW = 0,
    CCW
};

class Motor
{
public:
    TIM_HandleTypeDef Driver_PWM_TIM;
    uint8_t Driver_PWM_TIM_Channel_x;

    uint16_t Output_A_GPIO_Pin;
    GPIO_TypeDef *Output_A_GPIOx;
    uint16_t Output_B_GPIO_Pin;
    GPIO_TypeDef *Output_B_GPIOx;

    Rotate_Direction Rotate_Direction_Flag = CW;
    double Motor_Full_Omega = MOTOR_FULL_OMEGA;
    int32_t Motor_PWM_Period = MOTOR_CALCULATE_PRESCALER;
    int32_t Out = 0;

public:
    void Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x,
              uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx);

    void Set_Rotate_Direction_Flag(Rotate_Direction __Rotate_Direction_Flag);
    void Set_Motor_Full_Omega(double __Motor_Full_Omega);
    void Set_Motor_PWM_Period(int32_t __Motor_PWM_Period);
    void Set_Out(int32_t __Out);

    Rotate_Direction Get_Rotate_Direction_Flag();
    double Get_Motor_Full_Omega();
    int32_t Get_Motor_PWM_Period();
    int32_t Get_Out();

    void Output();
};

class Motor_Hall_Encoder : public Motor
{
public:
    PID Omega_PID;
    PID Angle_PID;

    TIM_HandleTypeDef Calculate_TIM;
    uint16_t Input_A_GPIO_Pin;
    GPIO_TypeDef *Input_A_GPIOx;
    uint16_t Input_B_GPIO_Pin;
    GPIO_TypeDef *Input_B_GPIOx;

    Control_Method Control_Method_Set = Control_Method_OMEGA;
    int32_t Hall_Encoder_Count = 0;
    int32_t Prev_Hall_Encoder_Count = 0;
    double Motor_Encoder_Num_Per_Rad = MOTOR_ENCODER_NUM_PER_RAD;
    double Omega_Now = 0;
    double Omega_Target = 0;
    double Angle_Now = 0;
    double Angle_Target = 0;

public:
    void Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x,
              uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx,
              TIM_HandleTypeDef __Calculate_EXTI_TIM,
              uint16_t __Input_A_GPIO_Pin, GPIO_TypeDef *__Input_A_GPIOx, uint16_t __Input_B_GPIO_Pin, GPIO_TypeDef *__Input_B_GPIOx);

    void Set_Control_Method(Control_Method Control_Method);
    void Set_Motor_Encoder_Num_Per_Rad(double Motor_Encoder_Num_Per_Rad);
    void Set_Omega_Target(double Omega_Target);
    void Set_Angle_Target(double Angle_Target);

    Control_Method Get_Control_Method();
    double Get_Omega_Now();
    double Get_Angle_Now();

    void Hall_Encoder_GPIO_EXTI_Callback();
    void Calculate_TIM_PeriodElapsedCallback();
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
void Move_Front(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Move_Back(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Move_Left(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Move_Right(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Rotate_Left(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Rotate_Right(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);
void Stop_Motor(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4);

#endif

/******************** COPYRIGHT(C) USTC-NATURALSELECTION **********************/
