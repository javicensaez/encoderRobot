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

			TxData[0] = (contDchTras& 0xFF00) >> 8;
			TxData[1] = contDchTras & 0x00FF;
			TxData[2] = (contIzqTras & 0xFF00) >> 8;
			TxData[3] = contIzqTras & 0x00FF;
			TxData[4] = (contDchDel & 0xFF00) >> 8;
			TxData[5] = contDchDel & 0x00FF;
			TxData[6] = (contIzqDel & 0xFF00) >> 8;
			TxData[7] = contIzqDel & 0x00FF;
			if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
					!= HAL_OK) {
				Error_Handler();
			}
}
