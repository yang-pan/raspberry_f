/*!******************************************************************************
 * @file    frizzDriver.h
 * @brief   source for frizzDriver
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __FRIZZ_DRIVER_H__
#define __FRIZZ_DRIVER_H__

#include "libsensors_id.h"
#include "hub_mgr_if.h"

#define D_FRIZZ_CHIPID	(0x00000200)	// frizz Chip ID(version)

#define D_FRIZZ_SENSOR_DEACTIVATE	(0)	// sensor is not active.
#define D_FRIZZ_SENSOR_ACTIVATE		(1)	// sensor is active.

/**
 *  Write firmware into frizz's I-ram 
 */
int frizzdrv_frizz_fw_download(const char* firmware_path );

/**
 *  Receive and analyze packet   
 */
int frizzdrv_receive_packet( void );

/**
 * Activate/Deactivate sensor
 * sen_id: id of the sensor to activate/deactivate
 * enabled: D_FRIZZ_SENSOR_DEACTIVATE: disable the sensor
 *          D_FRIZZ_SENSOR_ACTIVATE:   enable the sensor
 */
#define D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO	(0)			// use_fifo: sensor data is pushed to hw fifo
#define D_FRIZZ_ACTIVATE_PARAM_USE_SWFIFO	(1)			// use_fifo: sensor data is pushed to sw fifo
#define D_FRIZZ_ACTIVATE_PARAM_WITHOUT_INTERRUPT	(0) // use_int: output without interrupt signal
#define D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT		(1) // use_int: output with interrupt signal
int frizzdrv_activate( libsensors_id_e sen_id, int enabled, int use_fifo, int use_int );


/**
 * Set update time interval of sensor 
 * sen_id: id of the sensor to activate/deactivate
 * use_fifo:   
 * use_int:
 */
int frizzdrv_set_sensor_interval( libsensors_id_e sen_id, int interval, int use_fifo, int use_int );



/**
 * Activate the GPIO IRQ function of frizz (IRQ: frizz -> raspberry)
 * gpio_num:   gpio number (0~3)
 * gpio_level: 0: Active High, !0: Avtive Low
 */
#define D_FRIZZ_GPIO_INT_NUM_0	(0)	// gpio_num: use gpio number 0 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_1	(1)	// gpio_num: use gpio number 1 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_2	(2)	// gpio_num: use gpio number 2 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_3	(3)	// gpio_num: use gpio number 3 for interrupt
#define D_FRIZZ_INT_ACTIVE_HIGH	(0)	// gpio_level: Low before generate edge
#define D_FRIZZ_INT_ACTIVE_LOW	(1) // gpio_level: High before genearate edge
int frizzdrv_set_setting( unsigned int gpio_num, int gpio_level );

/**
 *  Get frizz version number from register 
 *  return value: version number (sucess)
 *                      -1       (failed)
 */
int frizzdrv_read_ver_reg( void );

#endif // __SENSOR_BUFF_H__
