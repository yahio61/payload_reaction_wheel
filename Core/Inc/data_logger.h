/*
 * data_logger.h
 *
 *  Created on: Jun 7, 2025
 *      Author: yahya
 */

#ifndef INC_DATA_LOGGER_H_
#define INC_DATA_LOGGER_H_
#include "stdint.h"

void data_logger_init();
void log_datas(uint32_t time, float angle_of_cylinder, int pwm, float error_angle);
#endif /* INC_DATA_LOGGER_H_ */
