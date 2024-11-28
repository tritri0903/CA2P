/*
 * sd_card.h
 *
 *  Created on: Nov 28, 2024
 *      Author: Tristan
 */

#include "main.h"

#ifndef SD_CARD_SD_CARD_H_
#define SD_CARD_SD_CARD_H_

int SD_Card_Init(void);
int SD_Card_Write_Test(void);
int SD_Card_Write(uint8_t *buf);
int SD_Card_Unmount(void);

#endif /* SD_CARD_SD_CARD_H_ */
