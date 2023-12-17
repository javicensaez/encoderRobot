#include "control_motores.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void motorIzqDel(int vel){


	if (vel>0){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); //IN1
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, vel); //IN2
	}else{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, vel); //IN1
	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0); //IN2
	}
}




void motorIzqTras(int vel){

	if (vel>0){
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, 0); //IN3
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, vel); //IN4
	}else{
	__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, vel); //IN3
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); //IN4
	}
}





void motorDchDel(int vel){

	if (vel>0){
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0); //IN5
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, vel); //IN6
	}else{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, vel); //IN5
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); //IN6
	}
}



void motorDchTras(int vel){

	if (vel>0){
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); //IN7
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, vel); //IN8
	}else{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, vel); //IN7
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); //IN8
	}
}

void initMotores(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //IN1
	HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1);//IN2
	HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);//IN3
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //IN4
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);//IN5
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //IN6
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //IN7
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //IN8
}
