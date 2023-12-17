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

void motorIzqDel(int vel);
void motorIzqTras(int vel);
void motorDchDel(int vel);
void motorDchTras(int vel);
void initMotores();

struct Motor {
  float kp;
  float ki;
  float kd;
  float velDeseada;
  float velMotor;
  float velMotorPasada;
};
