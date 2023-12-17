/*
 * encoder.c
 *
 *  Created on: Dec 17, 2023
 *      Author: javivi
 */
#include "encoder.h"

void contar() {
	int vENC1A = HAL_GPIO_ReadPin(ENC1A_GPIO_Port, ENC1A_Pin);
	int vENC1B = HAL_GPIO_ReadPin(ENC1B_GPIO_Port, ENC1B_Pin);
	int vENC2A = HAL_GPIO_ReadPin(ENC2A_GPIO_Port, ENC2A_Pin);
	int vENC2B = HAL_GPIO_ReadPin(ENC2B_GPIO_Port, ENC2B_Pin);
	int vENC3A = HAL_GPIO_ReadPin(ENC3A_GPIO_Port, ENC3A_Pin);
	int vENC3B = HAL_GPIO_ReadPin(ENC3B_GPIO_Port, ENC3B_Pin);
	int vENC4A = HAL_GPIO_ReadPin(ENC4A_GPIO_Port, ENC4A_Pin);
	int vENC4B = HAL_GPIO_ReadPin(ENC4B_GPIO_Port, ENC4B_Pin);
	static int vENC1AP;
	static int vENC2AP;
	static int vENC3AP;
	static int vENC4AP;

	if ((vENC1A == 1) && (vENC1AP == 0)) {
		if (vENC1B) {
			cont1++;
		} else {
			cont1--;
		}
	}

	if ((vENC2A == 1) && (vENC2AP == 0)) {
		if (vENC2B) {
			cont2++;
		} else {
			cont2--;
		}
	}

	if ((vENC3A == 1) && (vENC3AP == 0)) {
		if (vENC3B) {
			cont3++;
			cont3temp++;
		} else {
			cont3--;
			cont3temp--;
		}
	}

	if ((vENC4A == 1) && (vENC4AP == 0)) {
		if (vENC4B) {
			cont4++;
		} else {
			cont4--;
		}
	}

	vENC1AP = vENC1A;
	vENC2AP = vENC2A;
	vENC3AP = vENC3A;
	vENC4AP = vENC4A;

}

