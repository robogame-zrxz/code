/**
 * @file motor.cpp
 * @author lavinal austin712@mail.ustc.edu.cn
 * @brief motor
 * @version 0.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.hpp"
#include "gpio.h"
#include "tim.h"

/* Private macros ------------------------------------------------------------*/
#define ON (GPIO_PIN_SET)
#define OFF (GPIO_PIN_RESET)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Motor motor1;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief
 * @note
 *
 * @param
 * @return
 */

template <typename Type>
void Math_Constrain(Type *x, Type Min, Type Max)
{
	if (*x < Min)
	{
		*x = Min;
	}
	else if (*x > Max)
	{
		*x = Max;
	}
}

template <typename Type>
Type Math_Abs(Type x)
{
	return ((x > 0) ? x : -x);
}

void Motor::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x,
				 uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx)
{
	Driver_PWM_TIM = __Driver_PWM_TIM;
	Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
	Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
	Output_A_GPIOx = __Output_A_GPIOx;
	Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
	Output_B_GPIOx = __Output_B_GPIOx;

	HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

void Motor::Set_Rotate_Direction_Flag(Rotate_Direction __Rotate_Direction_Flag)
{
	Rotate_Direction_Flag = __Rotate_Direction_Flag;
}

void Motor::Set_Motor_Full_Omega(double __Motor_Full_Omega)
{
	Motor_Full_Omega = __Motor_Full_Omega;
}

void Motor::Set_Motor_PWM_Period(int32_t __Motor_PWM_Period)
{
	Motor_PWM_Period = __Motor_PWM_Period;
}

void Motor::Set_Out(int32_t __Out)
{
	Out = __Out;
}

Rotate_Direction Motor::Get_Rotate_Direction_Flag()
{
	return (Rotate_Direction_Flag);
}

double Motor::Get_Motor_Full_Omega()
{
	return (Motor_Full_Omega);
}

int32_t Motor::Get_Motor_PWM_Period()
{
	return (Motor_PWM_Period);
}

int32_t Motor::Get_Out()
{
	return (Out);
}

void Motor::Output()
{
	if (Out == 0)
	{
		HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, OFF);
		HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, OFF);
	}
	else if (Out > 0)
	{
		HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, ON);
		HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, OFF);
	}
	else if (Out < 0)
	{
		HAL_GPIO_WritePin(Output_A_GPIOx, Output_A_GPIO_Pin, OFF);
		HAL_GPIO_WritePin(Output_B_GPIOx, Output_B_GPIO_Pin, ON);
	}
	__HAL_TIM_SetCompare(&Driver_PWM_TIM, Driver_PWM_TIM_Channel_x, Math_Abs(Out));
}

void Motor_Hall_Encoder::Init(TIM_HandleTypeDef __Driver_PWM_TIM, uint8_t __Driver_PWM_TIM_Channel_x,
							  uint16_t __Output_A_GPIO_Pin, GPIO_TypeDef *__Output_A_GPIOx, uint16_t __Output_B_GPIO_Pin, GPIO_TypeDef *__Output_B_GPIOx,
							  TIM_HandleTypeDef __Calculate_TIM,
							  uint16_t __Input_A_GPIO_Pin, GPIO_TypeDef *__Input_A_GPIOx, uint16_t __Input_B_GPIO_Pin, GPIO_TypeDef *__Input_B_GPIOx)
{
	Driver_PWM_TIM = __Driver_PWM_TIM;
	Driver_PWM_TIM_Channel_x = __Driver_PWM_TIM_Channel_x;
	Output_A_GPIO_Pin = __Output_A_GPIO_Pin;
	Output_A_GPIOx = __Output_A_GPIOx;
	Output_B_GPIO_Pin = __Output_B_GPIO_Pin;
	Output_B_GPIOx = __Output_B_GPIOx;
	Calculate_TIM = __Calculate_TIM;
	Input_A_GPIO_Pin = __Input_A_GPIO_Pin;
	Input_A_GPIOx = __Input_A_GPIOx;
	Input_B_GPIO_Pin = __Input_B_GPIO_Pin;
	Input_B_GPIOx = __Input_B_GPIOx;
	HAL_TIM_PWM_Start(&__Driver_PWM_TIM, __Driver_PWM_TIM_Channel_x);
}

void Motor_Hall_Encoder::Set_Control_Method(Control_Method __Control_Method)
{
	Control_Method_Set = __Control_Method;
}

void Motor_Hall_Encoder::Set_Omega_Target(double __Omega_Target)
{
	Omega_Target = __Omega_Target;
}

void Motor_Hall_Encoder::Set_Motor_Encoder_Num_Per_Rad(double __Motor_Encoder_Num_Per_Rad)
{
	Motor_Encoder_Num_Per_Rad = __Motor_Encoder_Num_Per_Rad;
}

void Motor_Hall_Encoder::Set_Angle_Target(double __Angle_Target)
{
	Angle_Target = __Angle_Target;
}

Control_Method Motor_Hall_Encoder::Get_Control_Method()
{
	return (Control_Method_Set);
}

double Motor_Hall_Encoder::Get_Omega_Now()
{
	return (Omega_Now);
}

float Motor_Hall_Encoder::Get_Angle_Now()
{
	return (Angle_Now);
}

void Motor_Hall_Encoder::Hall_Encoder_GPIO_EXTI_Callback()
{
	if (((HAL_GPIO_ReadPin(Input_B_GPIOx, Input_B_GPIO_Pin) == 0) ^ (Rotate_Direction_Flag == CW)) == 0)
	{
		Hall_Encoder_Count--;
	}
	else
	{
		Hall_Encoder_Count++;
	}
}

void Motor_Hall_Encoder::Calculate_TIM_PeriodElapsedCallback()
{
	int32_t delta;
	delta = Hall_Encoder_Count - Prev_Hall_Encoder_Count;
	Angle_Now += (double)delta / Motor_Encoder_Num_Per_Rad;
	Omega_Now = (double)delta / Motor_Encoder_Num_Per_Rad / MOTOR_CALCULATE_PERIOD;
	Prev_Hall_Encoder_Count = Hall_Encoder_Count;

	if (Control_Method_Set == Control_Method_OPENLOOP)
	{
		Out = Omega_Target * Motor_PWM_Period / Motor_Full_Omega;
	}
	else if (Control_Method_Set == Control_Method_OMEGA)
	{
		Omega_PID.Set_Now(Omega_Now);
		Omega_PID.Set_Target(Omega_Target);
		Omega_PID.Adjust_TIM_PeriodElapsedCallback();
		Out = Omega_PID.Get_Out();
	}
	else if (Control_Method_Set == Control_Method_ANGLE)
	{
		Angle_PID.Set_Target(Angle_Target);

		Angle_PID.Set_Now(Angle_Now);
		Angle_PID.Adjust_TIM_PeriodElapsedCallback();
		Omega_PID.Set_Target(Angle_PID.Get_Out());

		Omega_PID.Set_Now(Omega_Now);
		Omega_PID.Adjust_TIM_PeriodElapsedCallback();
		Out = Omega_PID.Get_Out();
	}
	Math_Constrain(&Out, -MOTOR_CALCULATE_PRESCALER, MOTOR_CALCULATE_PRESCALER);
	Output();
}

void Move_Front(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.clockwise_rotation();
	motor2.clockwise_rotation();
	motor3.anticlockwise_rotation();
	motor4.anticlockwise_rotation();
}

void Move_Back(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.anticlockwise_rotation();
	motor2.anticlockwise_rotation();
	motor3.clockwise_rotation();
	motor4.clockwise_rotation();
}

void Move_Left(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.anticlockwise_rotation();
	motor2.clockwise_rotation();
	motor3.clockwise_rotation();
	motor4.anticlockwise_rotation();
}

void Move_Right(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.clockwise_rotation();
	motor2.anticlockwise_rotation();
	motor3.anticlockwise_rotation();
	motor4.clockwise_rotation();
}

void Rotate_Left(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.anticlockwise_rotation();
	motor2.anticlockwise_rotation();
	motor3.anticlockwise_rotation();
	motor4.anticlockwise_rotation();
}

void Rotate_Right(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.clockwise_rotation();
	motor2.clockwise_rotation();
	motor3.clockwise_rotation();
	motor4.clockwise_rotation();
}

void Stop_Motor(Motor &motor1, Motor &motor2, Motor &motor3, Motor &motor4)
{
	motor1.stop();
	motor2.stop();
	motor3.stop();
	motor4.stop();
}

/******************** COPYRIGHT(C) USTC-NATURALSELECTION **********************/
