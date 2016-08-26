/*!******************************************************************************
 * @file    bmd101_sensor_writer.h
 * @brief   source for bmd101_sensor_writer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __BMD101_SENSOR_WRITER_H__
#define __BMD101_SENSOR_WRITER_H__

#include <stdio.h>
#include "sensor_buff.h"

void BMD101SensorWriter(sensor_data_t* recv_data, FILE* fp);
#endif	// __BMD101_SENSOR_WRITER_H__
