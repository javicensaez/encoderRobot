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

extern int16_t contIzqTras;
extern int16_t contIzqDel;
extern int16_t contDchDel;
extern  int16_t contDchTras;


void enviarDatosCan();
