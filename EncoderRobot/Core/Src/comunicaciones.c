/*
 * comunicaciones.c
 *
 *  Created on: Dec 17, 2023
 *      Author: javivi
 */

#include "comunicaciones.h"

void enviarDatosCan(){
	TxHeader.ExtId = 0x0CF11E05; //ID extendida 1
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.DLC = 8;
			TxHeader.TransmitGlobalTime = DISABLE;

			TxData[0] = (cont1 & 0xFF00) >> 8;
			TxData[1] = cont1 & 0x00FF;
			TxData[2] = (cont2 & 0xFF00) >> 8;
			TxData[3] = cont2 & 0x00FF;
			TxData[4] = (cont3 & 0xFF00) >> 8;
			TxData[5] = cont3 & 0x00FF;
			TxData[6] = (cont4 & 0xFF00) >> 8;
			TxData[7] = cont4 & 0x00FF;
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
					!= HAL_OK) {
				//Error_Handler();
			}
}
