/*
 * encoder.h
 *
 *  Created on: Dec 17, 2023
 *      Author: javivi
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_



#endif /* INC_ENCODER_H_ */

#include "stm32f3xx_hal.h"
#include "main.h"

extern int16_t contIzqTras;
extern float contIzqTrasMedia;
extern float contIzqTrasTemp;
extern int16_t contIzqDel;
extern float contIzqDelMedia;
extern float contIzqDelTemp;
extern int16_t contDchDel;
extern float contDchDelMedia;
extern float contDchDelTemp;
extern int16_t contDchTras;
extern float contDchTrasMedia;
extern float contDchTrasTemp;

extern float rpmIzqTras;
extern float rpmIzqDel;
extern float rpmDchDel;
extern float rpmDchTras;

extern float filtRpmIzqTras;
extern float  filtRpmIzqDel;
extern float  filtRpmDchDel;
extern float  filtRpmDchTras;

void contar();
void calculaRPM();
