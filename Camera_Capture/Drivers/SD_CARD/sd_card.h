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
int SD_Card_Open(const char *file);
int SD_Card_Write_Log(const char* msg);
int SD_Card_Write(uint8_t *buf);
int SD_Card_Unmount(void);
int SD_Card_Save_Temp(uint8_t* buf);
void SD_Card_Close(void);

#endif /* SD_CARD_SD_CARD_H_ */
