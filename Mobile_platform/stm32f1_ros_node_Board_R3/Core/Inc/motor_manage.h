/***
@Author: Vincent Chan
@About: Motor Control libraries
***/
#include "main.h"

/*** Enabled peripherals ***/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

/*** User defined variables ***/
const int max_register_value = 2000;
enum MOTOR_INDEX{MOTOR1,MOTOR2,MOTOR3,MOTOR4};
enum MOTOR_DIRECTION{CLOCKWISE,COUNTERCLOCKWISE,BREAK};

/*** Function Prototypes ***/
void motor_peripherals_init(MOTOR_INDEX motor);
void motor_controller(MOTOR_INDEX motor,MOTOR_DIRECTION turn,int duty_cycle, int max_counter);
int calculate_MOTOR_timer_register(int percentage,double counter_max);
void motor_selftest(void);

/*** Function Definition ***/
// 1. Enable motor peripherals based on MOTOR_INDEX
void motor_peripherals_init(MOTOR_INDEX motor)
{
	int x = 0;
	if(motor == MOTOR1)
	{
		// MOTOR1 with ENCODER 3
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
		HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	}else if(motor == MOTOR2)
	{
		// MOTOR2 with ENCODER 4 [Align with BAT IN in the motor drive]
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
		HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	}else if (motor == MOTOR3)
	{
		// TO-DO: Configure motor 3
		x = 0;
	}else{
		// TO-DO: Configure motor 4
		x = 0;
	}
	x++;
}

// 2. Motor Controller
void motor_controller(MOTOR_INDEX motor,MOTOR_DIRECTION turn,int duty_cycle,int max_counter)
{
	int x = 0;
	if(motor == MOTOR1)
	{
		if(turn == CLOCKWISE)
		{
			HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);
		}else if (turn == COUNTERCLOCKWISE)
		{
			HAL_GPIO_WritePin(MOTOR1_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR1_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_SET);
		}else
		{
			HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR1_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR1_IN2_Pin,GPIO_PIN_RESET);			
		}
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,calculate_MOTOR_timer_register(duty_cycle,max_counter));
	}else if(motor == MOTOR2)
	{
		if(turn == CLOCKWISE)
		{
			HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);
		}else if (turn == COUNTERCLOCKWISE)
		{
			HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_SET);
		}else
		{
			HAL_GPIO_WritePin(MOTOR2_IN1_GPIO_Port,MOTOR2_IN1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR2_IN2_GPIO_Port,MOTOR2_IN2_Pin,GPIO_PIN_RESET);			
		}
	  __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,calculate_MOTOR_timer_register(duty_cycle,max_counter));
	}else if (motor == MOTOR3)
	{
		// TO-DO: Configure motor 3
		x = 0;
	}else{
		// TO-DO: Configure motor 4
		x = 0;
	}
	x++;
}	

// 3. Convert duty cycle to counter register value
int calculate_MOTOR_timer_register(int percentage,double counter_max)
{
	return (int)((percentage*counter_max)/100);
}

// 4. Motor Testing
void motor_selftest(void)
{
	// No debouncing because it is just for testing
	// A.When SW1 is pressed, the motors should turn in clockwise
	// B.When SW2 is pressed, the motors should turn in counterclockwise
	// C.When SW3 is pressed, the motors should stop
	if(HAL_GPIO_ReadPin(USER_SWITCH_1_GPIO_Port,USER_SWITCH_1_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,CLOCKWISE,10,max_register_value);
		motor_controller(MOTOR2,CLOCKWISE,10,max_register_value);
	}else if(HAL_GPIO_ReadPin(USER_SWITCH_2_GPIO_Port,USER_SWITCH_2_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,COUNTERCLOCKWISE,10,max_register_value);
		motor_controller(MOTOR2,COUNTERCLOCKWISE,10,max_register_value);		
	}else if(HAL_GPIO_ReadPin(USER_SWITCH_3_GPIO_Port,USER_SWITCH_3_Pin) == GPIO_PIN_RESET)
	{
		motor_controller(MOTOR1,BREAK	,0,max_register_value);
		motor_controller(MOTOR2,BREAK,0,max_register_value);			
	}else{
		int p = 0;
		p ++;
	}	
}