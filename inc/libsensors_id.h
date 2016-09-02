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
	SENSOR_ID_ACCEL_RAW					= 0x80,	///< need implementation at each System
	SENSOR_ID_MAGNET_INTERNAL			= 0xCB, ///< magnet sensor (uncalibration data + calibration data)
	SENSOR_ID_GYRO_RAW					= 0x82,	///< need implementation at each System
	SENSOR_ID_PRESSURE_RAW				= 0x83,	///< need implementation at each System
	SENSOR_ID_LIGHT_RAW					= 0xA9,	///< need implementation at each System
	SENSOR_ID_PROXIMITY_RAW				= 0xAA,	///< need implementation at each System
	SENSOR_ID_PPG_RAW					= 0xAC,	///< need implementation at each System
	SENSOR_ID_OELD_RAW					= 0xAD,	///< need implementation at each System
	SENSOR_ID_ECG_RAW					= 0xB1,	///< need implementation at each System
	SENSOR_ID_ADC_RAW					= 0xC2,	///< need implementation at each System
	SENSOR_ID_HUMIDITY_RAW				= 0xC3,	///< need implementation at each System
	SENSOR_ID_SPO2_RAW					= 0xC6,	///< need implementation at each System
	SENSOR_ID_TEMPERATURE_RAW			= 0xCC,	///< need implementation at each System
	/* Application Sensors */
	// Accelerometer
	SENSOR_ID_ACCEL_POWER				= 0x84,	///< accel power sensor
	SENSOR_ID_ACCEL_LPF					= 0x85,	///< accel LPF simple gravity direction
	SENSOR_ID_ACCEL_HPF					= 0x86,	///< accel HPF simple linear accel
	SENSOR_ID_ACCEL_STEP_DETECTOR		= 0x87,	///< accel step detection
	SENSOR_ID_ACCEL_PEDOMETER			= 0x88,	///< accel pedometer
	SENSOR_ID_ACCEL_LINEAR				= 0x89,	///< accel linear
	// Magnetometer
	SENSOR_ID_MAGNET_CALIB				= 0x81,	///< magnet sensor (calibration data)
	SENSOR_ID_MAGNET_PARAMETER			= 0x8A,	///< magnet calibration parameter
	SENSOR_ID_MAGNET_CALIB_SOFT			= 0x8B,	///< Soft Iron magnet sensor
	SENSOR_ID_MAGNET_CALIB_HARD			= 0x8C,	///< Soft Iron + Hard Iron magnet sensor
	SENSOR_ID_MAGNET_LPF				= 0x8D,	///< magnet LPF
	SENSOR_ID_MAGNET_UNCALIB			= 0x8E,	///< magnet sensor (uncalibration data)
	SENSOR_ID_MAGNET_CALIB_STATUS		= 0xAB,	///< calibration status + calibration parameter
	// Gyroscope
	SENSOR_ID_GYRO_LPF					= 0x8F,	///< gyro LPF => simple offset
	SENSOR_ID_GYRO_HPF					= 0x90,	///< gyro HPF => simple offset calibration
	SENSOR_ID_GYRO_UNCALIB				= 0x91,	///< gyro uncalibration + offset value
	// Pressure
	SENSOR_ID_PRES_LPF					= 0xD5,	///< pressure LPF => low frequency component
	// Fusion 6D
	SENSOR_ID_GRAVITY					= 0x92,	///< gravity depends on accel + gyro sensor
	SENSOR_ID_GRAVITY_AM				= 0xD8,	///< gravity depends on accel + magnet sensor
	SENSOR_ID_DIRECTION					= 0x93,	///< UnSupported!! magnetic north direction depends on accel + gyro sensor
	// Fusion 9D
	SENSOR_ID_POSTURE					= 0x94,	///< UnSupported!! gravity + magnetic north direction depends on accel + magnet + gyro sensor
	SENSOR_ID_ROTATION_MATRIX			= 0x95,	///< UnSupported!! use prohibition (rotation matrix: sensor coordinate system => world coordinate system[ENU])
	SENSOR_ID_ORIENTATION				= 0x96,	///< azimuth, pitch, roll
	SENSOR_ID_ROTATION_VECTOR			= 0x97,	///< four dimension + precision
	// PDR
	SENSOR_ID_PDR						= 0x98,	///< pedestrian dead reckoning sensor
	SENSOR_ID_VELOCITY					= 0x99,	///< UnSupported!! velocity sensor
	SENSOR_ID_RELATIVE_POSITION			= 0x9A, ///< UnSupported!! relative position sensor
	SENSOR_ID_MIGRATION_LENGTH			= 0x9B,	///< UnSupported!! migration length sensor
	// Util
	SENSOR_ID_CYCLIC_TIMER				= 0x9C,	///< cyclic timer sensor
	SENSOR_ID_DEBUG_QUEUE_IN			= 0x9D,	///< QUEUE input for DEBUG
	SENSOR_ID_DEBUG_STD_IN				= 0x9E,	///< standard input
	// Accelerometer
	SENSOR_ID_ACCEL_MOVE				= 0x9F,	///< accel move detection sensor, >>Internal ID
	// Libs
	SENSOR_ID_ISP						= 0xA0,	///< ISP PRESENCE sensor
	// Rotation
	SENSOR_ID_ROTATION_GRAVITY_VECTOR	= 0xA1,	///< Gravity Rotation Vector
	SENSOR_ID_ROTATION_LPF_VECTOR		= 0xA2,	///< Rotation Vector without Gyro
	/* Fall down detection */
	SENSOR_ID_ACCEL_FALL_DOWN			= 0xA3,	///< Fall down sensor, >>Internal ID
	SENSOR_ID_ACCEL_POS_DET				= 0xA4,	///< posture detection sensor
	/* Geofencing */
	SENSOR_ID_PDR_GEOFENCING			= 0xA5,	///< geofencing sensor
	/* Gesture */
	SENSOR_ID_GESTURE					= 0xA6,	///< gesture sensor
	//SENSOR_ID_GESTURE_RESULT			= 0xA7,	///< gesture result sensor
	/* Stair detector */
	SENSOR_ID_STAIR_DETECTOR			= 0xA8, ///< Stair detector sensor

	/* blood pressure */
	SENSOR_ID_HEART_RATE_PIXART			= 0xC0,	///< customer heart rate
	SENSOR_ID_HEART_RATE_NEUROSKY		= 0xC1,	///< customer heart rate
	SENSOR_ID_ECG_HRM					= 0xC7,	///< customer heart rate

	SENSOR_ID_BLOOD_PRESSURE			= 0xAE,	///< blood_pressure sensor
	SENSOR_ID_BLOOD_PRESSURE_LEARN		= 0xAF,	///< blood_pressure sensor
	SENSOR_ID_HEART_RATE				= 0xCA, ///< heart rate sensor(HAL interface)
	SENSOR_ID_STRESS_MEASURE			= 0xC9,	///< Stress measurement
	SENSOR_ID_PPG_QUALITY_CHECK			= 0xCE, ///< ppg data quality judgement

	/* wearing, touch, tap */
	SENSOR_ID_WEARING_DETECTOR			= 0xB0,	///< wearing detect sensor
	SENSOR_ID_TOUCH_DETECTOR			= 0xC8,	///< touch detect sensor
	SENSOR_ID_TAP_DETECTOR				= 0xCD,	///< tap detect sensor
	SENSOR_ID_SMART_WEARING_DETECTOR    = 0xD3, ///< pixart touch flag + ppg quality check
	SENSOR_ID_HIGH_G_DETECTOR			= 0xD4,	///< highG detect sensor

	/* Android Gesture */
	SENSOR_ID_ACCEL_STATISTICS			= 0xB2,	///< accel stat sensor
	SENSOR_ID_GLANCE_STATUS				= 0xB3,	///< glance status sensor
	SENSOR_ID_TILT_DETECTOR				= 0xB4,	///< android tilt detector sensor
	SENSOR_ID_PICKUP_DETECTOR			= 0xB5,	///< android pickup gesture sensor
	SENSOR_ID_GLANCE_DETECTOR			= 0xB6,	///< android glance gesture sensor
	SENSOR_ID_WAKEUP_DETECTOR			= 0xB7,	///< android wakeup gesture sensor
	SENSOR_ID_WRIST_TILT_DETECTOR		= 0xB8,	///< android wrist tilt gesture sensor
	/* Proximity application */
	SENSOR_ID_PROXIMITY_DETECTOR		= 0xC4, ///< proximity detect sensor

	/* Activity Detection */
	SENSOR_ID_ACTIVITY_DETECTOR			= 0xBA,	///< activity detection sensor
	/* Motion Detection */
	SENSOR_ID_STEP_TRACKER				= 0xBB,	///< step tracker sensor
	SENSOR_ID_MOTION_DETECTOR			= 0xBC,	///< motion detection sensor, >>Internal ID
	SENSOR_ID_MOTION_SENSING			= 0xBD,	///< motion sensing sensor
	SENSOR_ID_CALORIE					= 0xBF,	///< calorie sensor via Pedometer
	/* BikeDetection */
	SENSOR_ID_BIKE_DETECTOR				= 0xBE,	///< bike detection sensor
	/* HybridGPS */
	SENSOR_ID_DR						= 0xD0,	///< dr sensor
	SENSOR_ID_HYBRIDGPS					= 0xD1,  ///< hybrid gps module
} libsensors_id_e;


/**
 * @enum lib_sensor_type_e
 * @brief Sensor ID
 */
typedef enum {
    //FRIZZ_SENSOR_TYPE_META_DATA                     = 0x00,
    FRIZZ_SENSOR_TYPE_ACCELEROMETER                 = 0x01, ///< Acceleration values in SI units (m/s^2)
    FRIZZ_SENSOR_TYPE_MAGNETIC_FIELD                = 0x02, ///< calibrated magnet data + calibration status
    //FRIZZ_SENSOR_TYPE_ORIENTATION                   = 0x03,
    FRIZZ_SENSOR_TYPE_GYROSCOPE                     = 0x04, ///< GYRO sensor(with zero offset calibration)
    FRIZZ_SENSOR_TYPE_LIGHT                         = SENSOR_ID_LIGHT_RAW,
    FRIZZ_SENSOR_TYPE_PRESSURE                      = SENSOR_ID_PRESSURE_RAW,
    //FRIZZ_SENSOR_TYPE_TEMPERATURE                   = 0x07,
    FRIZZ_SENSOR_TYPE_PROXIMITY                     = SENSOR_ID_PROXIMITY_DETECTOR,
    FRIZZ_SENSOR_TYPE_GRAVITY                       = 0x09, ///< gravity valudes(m/s^2)+status(dummy)
    FRIZZ_SENSOR_TYPE_LINEAR_ACCELERATION           = 0x0A, ///< linear acceleration(m/s^2) without gravity
    FRIZZ_SENSOR_TYPE_ROTATION_VECTOR               = SENSOR_ID_ROTATION_VECTOR,
    FRIZZ_SENSOR_TYPE_RELATIVE_HUMIDITY             = SENSOR_ID_HUMIDITY_RAW,
    FRIZZ_SENSOR_TYPE_AMBIENT_TEMPERATURE           = 0x0D, ///< ambient tempearture(degree Celsius)
    //FRIZZ_SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED   = 0x0E,
    FRIZZ_SENSOR_TYPE_GAME_ROTATION_VECTOR          = SENSOR_ID_ROTATION_GRAVITY_VECTOR,
    FRIZZ_SENSOR_TYPE_GYROSCOPE_UNCALIBRATED        = SENSOR_ID_GYRO_UNCALIB,
    FRIZZ_SENSOR_TYPE_SIGNIFICANT_MOTION            = SENSOR_ID_ACCEL_MOVE,
    FRIZZ_SENSOR_TYPE_STEP_DETECTOR                 = SENSOR_ID_ACCEL_STEP_DETECTOR,
    FRIZZ_SENSOR_TYPE_STEP_COUNTER                  = 0x13, ///< step counter(64bit)
    FRIZZ_SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR   = SENSOR_ID_ROTATION_LPF_VECTOR,
    FRIZZ_SENSOR_TYPE_HEART_RATE                    = SENSOR_ID_HEART_RATE,
    FRIZZ_SENSOR_TYPE_TILT_DETECTOR                 = SENSOR_ID_TILT_DETECTOR,
    //FRIZZ_SENSOR_TYPE_WAKE_GESTURE                  = 0x17, 
    FRIZZ_SENSOR_TYPE_GLANCE_GESTURE                = SENSOR_ID_GLANCE_DETECTOR,
    FRIZZ_SENSOR_TYPE_PICK_UP_GESTURE               = SENSOR_ID_PICKUP_DETECTOR,
    FRIZZ_SENSOR_TYPE_WRIST_TILT_GESTURE            = SENSOR_ID_WRIST_TILT_DETECTOR,
}lib_sensor_type_e;



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
 * @param cmd_param[2] Assign z-axis
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
