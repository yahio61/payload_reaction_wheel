/*
 * data_logger.c
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */
#include "data_logger.h"
#include "fatfs.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"

FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; 	//Result after operations

//For file operation functions look at https://elm-chan.org/fsw/ff/00index_e.html

void data_logger_init()
{
	fres = f_mount(&FatFs, "", 1);
	fres = f_open(&fil, "paylod.csv", FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&fil, f_size(&fil));
	unsigned int file_res = 0;
	uint8_t p_data[300];
	sprintf((char*) p_data, (char*)"time,angle_of_cylinder,pwm,error_angle\n");
	f_write(&fil, (uint8_t*) p_data, strlen((char*)p_data), &file_res);
	f_close(&fil);
}

void log_datas(uint32_t time, float angle_of_cylinder, int pwm, float error_angle)
{
	fres = f_open(&fil, "paylod.csv", FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&fil, f_size(&fil));
	unsigned int file_res = 0;
	uint8_t p_data[300];
	sprintf((char*) p_data, (char*)"%lu,%f,%d,%f\n", time, angle_of_cylinder, pwm, error_angle);
	f_write(&fil, (uint8_t*) p_data, strlen((char*)p_data), &file_res);
	f_close(&fil);
}
