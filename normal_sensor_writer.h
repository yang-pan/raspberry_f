/*!******************************************************************************
 * @file    normal_sensor_writer.h
 * @brief   source for normal_sensor_writer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __NORMAL_SENSOR_WRITER_H__
#define __NORMAL_SENSOR_WRITER_H__

#include <stdio.h>
#include "common.h"
#include "sensor_buff.h"

void ucharWriter(const unsigned char* write_data, unsigned char length, FILE* fp);
unsigned char getSensorDataLength(sensor_data_t* recv_data);
unsigned char calcCheckSum(sensor_data_t* recv_data);
void normalSensorWriter(sensor_data_t* recv_data, FILE* fp);
#endif	// __NORMAL_SENSOR_WRITER_H__
