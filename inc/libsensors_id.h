/*!******************************************************************************
 * @file    libsensors_id.h
 * @brief   list of sensor ID
 * @par     Copyright
 *          (C) 2014 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *
 * This file may not be distributed, copied, or reproduced in any manner,
 * electronic or otherwise, without the written consent of MegaChips Corporation.
 *******************************************************************************/
#ifndef __LIBSENSORS_ID_H__
#define __LIBSENSORS_ID_H__

#ifdef __cplusplus
extern "C" {
#endif

	/**
	 * @enum libsensors_id_e
	 * @brief Sensor ID
	 */
	typedef enum {
		/* Physical Sensors */
		SENSOR_ID_ACCEL_RAW                 = 0x80, ///< need implementation at each System
		SENSOR_ID_MAGNET_RAW                = 0x81, ///< need implementation at each System
		SENSOR_ID_GYRO_RAW                  = 0x82, ///< need implementation at each System
		SENSOR_ID_PRESSURE_RAW              = 0x83, ///< need implementation at each System
		SENSOR_ID_ECG_RAW                   = 0xB1, ///< need implementation at each System
		/* Application Sensors */
		// Accelerometer
		SENSOR_ID_ACCEL_POWER               = 0x84, ///< accel power sensor
		SENSOR_ID_ACCEL_LPF                 = 0x85, ///< accel LPF simple gravity direction
		SENSOR_ID_ACCEL_HPF                 = 0x86, ///< accel HPF simple linear accel
		SENSOR_ID_ACCEL_STEP_DETECTOR       = 0x87, ///< accel step detection
		SENSOR_ID_ACCEL_PEDOMETER           = 0x88, ///< accel pedometer
		SENSOR_ID_ACCEL_LINEAR              = 0x89, ///< accel linear
		/* Magnetometer */
		SENSOR_ID_MAGNET_PARAMETER          = 0x8A, ///< magnet calibration parameter
		SENSOR_ID_MAGNET_CALIB_SOFT         = 0x8B, ///< Soft Iron magnet sensor
		SENSOR_ID_MAGNET_CALIB_HARD         = 0x8C, ///< Soft Iron + Hard Iron magnet sensor
		SENSOR_ID_MAGNET_LPF                = 0x8D, ///< magnet LPF
		SENSOR_ID_MAGNET_UNCALIB            = 0x8E, ///< Soft Iron + Hard Iron parameter
		SENSOR_ID_MAGNET_CALIB_RAW          = 0xAB,  ///< Iron magnet sensor raw calibration
		/* Gyroscope */
		SENSOR_ID_GYRO_LPF                  = 0x8F, ///< gyro LPF => simple offset
		SENSOR_ID_GYRO_HPF                  = 0x90, ///< gyro HPF => simple offset calibration
		SENSOR_ID_GYRO_UNCALIB              = 0x91, ///< gyro uncalibration + offset value
	} libsensors_id_e;

	/** @defgroup GENERALCOMMAND General for Sensor Libraries
	 *  @brief General contents for sensor libraries
	 *  @{
	 */
	/**
	 * @name Command List
	 */
	//@{
	/**
	 * @brief Get sensor library version (for all sensor libraries).
	 * @brief Version is gotten by receive data.
	 * @param cmd_code SENSOR_GET_VERSION
	 * @return [unsigned int] version (The first 2bytes are major vesion. The last 2bytes are minor vesion.)
	 *
	 */
#define	SENSOR_GET_VERSION				(0xFF)

	/**
	 * @brief Get sensor name correspond to specified physical sensor library.
	 * @brief Device name is gotten by receive data.
	 * @param cmd_code DEVICE_GET_NAME
	 * @return [unsigned int] device name (It is necessary to change ASCII code.)
	 *
	 */
#define	DEVICE_GET_NAME					(0xFE)

	/**
	 * @brief Set sensor direction (for 3-axis sensor).
	 * @brief Setting result is gotten by receive data.\n
	 * @brief command_param[0]~[2]  0x00:x-axis 0x01:y-axis 0x02:z-axis (When directions overlap, setting is not successful.)\n
	 * @brief command_param[3]~[5]  0x00:Change from plus direction to plus directon 0x01:Change from minus direction to plus directon
	 * @param cmd_code SENSOR_SET_DIRECTION
	 * @param cmd_param[0] Assign x-axis
	 * @param cmd_param[1] Assign y-axis
	 * @param cmd_param[2] Assign y-axis
	 * @param cmd_param[3] Plus direction of x-axis
	 * @param cmd_param[4] Plus direction of y-axis
	 * @param cmd_param[5] Plus direction of z-axis
	 * @return [int] -1:Fail, Others: Success
	 *
	 */
#define	SENSOR_SET_DIRECTION			(0xFD)
	//@}
	/** @} */


#define INNER_SENSOR_CMD_CODE_TO_CMD(cmd_code)	(unsigned long)(((cmd_code)<<24)&0xFFFFFFFF) ///< Command conversion (only inner sensor)
#define SENSOR_MGR_CMD_CODE_TO_CMD(cmd_code)	(unsigned char)(((cmd_code)>>24)&0xFF)		 ///< Command conversion

#ifdef __cplusplus
}
#endif

#endif
