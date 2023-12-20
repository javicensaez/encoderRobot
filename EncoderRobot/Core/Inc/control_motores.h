/*
 * control_motores.h
 *
 *  Created on: Dec 17, 2023
 *      Author: javivi
 */

#ifndef INC_CONTROL_MOTORES_H_
#define INC_CONTROL_MOTORES_H_



#endif /* INC_CONTROL_MOTORES_H_ */

#include "stm32f3xx_hal.h"

struct pid_motor {
  float kp;
  float ki;
  float kd;
  float velDeseada;
  float velMotor;
  float errorPasado;
  float sum;
  float delta;
};

extern float rpmIzqTras;

extern struct pid_motor PIDmotorIzqTras;

void motorIzqDel(int vel);
void motorIzqTras(int vel);
void motorDchDel(int vel);
void motorDchTras(int vel);
void initMotores();
void pid_controller_init(struct pid_motor *pid, float delta, float kp, float ki, float kd);
void calculaPID();

void updateMotor();
float pid_controller_run(struct pid_motor *pid, float velMotor);
