/*
 * sd_card.c
 *
 *  Created on: Nov 28, 2024
 *      Author: Tristan
 */

#include "sd_card.h"
#include "fatfs.h"

extern UART_HandleTypeDef huart2;

FATFS FatFs;
FIL Fil;
FRESULT FR_Status;
FATFS *FS_Ptr;
UINT RWC, WWC; // Read/Write Word Counter
DWORD FreeClusters;
uint32_t TotalSize, FreeSpace;
char RW_Buffer[1024];

int SD_Card_Init(void) {
	//------------------[ Mount The SD Card ]--------------------
	FR_Status = f_mount(&FatFs, "", 1);
	HAL_Delay(500);
	if (FR_Status != FR_OK) {
		return 1;
	} else {
		return 0;
	}
}

int SD_Card_Write_Log(const char *msg) {
	SD_Card_Open("Log.txt");
	// (2) Write Data To The Text File
	f_write(&Fil, msg, strlen(msg), &WWC);
	return 0;
}

int SD_Card_Save_Temp(uint8_t* buf) {
	FR_Status = f_open(&Fil, "Temp.jpg", FA_WRITE | FA_CREATE_ALWAYS);
	f_write(&Fil, buf, sizeof(buf), &WWC);
	f_close(&Fil);
	return 0;
}

int SD_Card_Open(const char *file) {
	FR_Status = f_open(&Fil, file, FA_WRITE | FA_OPEN_APPEND);
	return FR_Status;
}

int SD_Card_Write(uint8_t* buf, uint16_t len) {
	// Write Data To The Text File
	f_write(&Fil, buf, len, &WWC);
	return WWC;
}
int SD_Card_FreeSpace(void){
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    return FreeSpace;
}
void SD_Card_Close(void){
	// Close The File
	f_close(&Fil);
}

int SD_Card_Unmount(void) {
	//------------------[ Unmount The SD Card ]--------------------
	FR_Status = f_mount(NULL, "", 0);
	if (FR_Status != FR_OK) {
		return 1;
	} else {
		return 0;
	}
}

