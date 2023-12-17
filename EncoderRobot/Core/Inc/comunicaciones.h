/*
 * comunicaciones.h
 *
 *  Created on: Dec 17, 2023
 *      Author: javivi
 */

#ifndef INC_COMUNICACIONES_H_
#define INC_COMUNICACIONES_H_



#endif /* INC_COMUNICACIONES_H_ */

#include "stm32f3xx_hal.h"

extern CAN_HandleTypeDef hcan;
extern uint8_t RxData[8];
extern uint8_t TxData[8];
extern CAN_RxHeaderTypeDef RxHeader;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailbox;

extern int16_t cont1;
extern int16_t cont2;
extern int16_t cont3;
extern float cont3medi;
extern float cont3tem;
extern  int16_t cont4;


void enviarDatosCan();
