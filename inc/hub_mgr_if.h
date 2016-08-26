/*!******************************************************************************
 * @file  hub_mgr_if.h
 * @brief source for hub manager interface
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
#ifndef __HUB_MGR_IF_H__
#define __HUB_MGR_IF_H__

/**
 * @name Version
 */
//@{
#define HUB_MGR_VERSION	(20150713)
//@}

/**
 * @name Sensor ID
 */
//@{
/**
 * @brief HUB Manager's Sensor ID
 */
#define HUB_MGR_ID	0xFF
//@}

/**
 * @name Output
 */
//@{
/**
 * @brief output data
 * @note On-changed
 * @note this sensor always pushes to HW FIFO
 */
typedef struct
{
	int	interval;		/// Interval Time [msec] 0: On-event, 0<: interval time
	int f_status;		/// status flag #HUB_MGR_IF_STATUS_ACTIVE ...
} hub_mgr_data_t;
//@}

/**
 * @name Status Flag
 */
//@{
#define HUB_MGR_IF_STATUS_ACTIVE									(1 << 0)	/// some sensor is active
#define HUB_MGR_IF_STATUS_FIFO_FULL_NOTIFY							(1 << 1)	/// detect SW FIFO threshold
#define HUB_MGR_IF_STATUS_FIFO_FULL_NOTIFY_SECONDARY				(1 << 2)	/// detect SW FIFO secondary threshold
#define HUB_MGR_IF_STATUS_FIFO_LATEST_DATA_AVAILABLE				(1 << 3)	/// Dumped specified sensor data
#define HUB_MGR_IF_STATUS_FIFO_LATEST_DATA_AVAILABLE_SECONDARY		(1 << 4)	/// Dumped specified secondary sensor data
#define HUB_MGR_IF_STATUS_UPDATED									(1 <<30)	/// Flag of updated (for Internal)
#define HUB_MGR_IF_STATUS_END										(1 <<31)	/// Force stop (for DEBUG)
//@}

/**
 * @name Command Code Generator
 */
//@{
/**
 * @brief Generate Command Code for HUB Manager
 *
 * @param cmd_id: command ID
 * @param imm0: immediate value0
 * @param imm1: immediate value1
 * @param imm2: immediate value2
 *
 * @return Command Code for HUB Manager
 */
#define HUB_MGR_GEN_CMD_CODE(cmd_id, imm0, imm1, imm2)	\
	(unsigned int)( (((cmd_id)&0xFF)<<24) | (((imm0)&0xFF)<<16) | (((imm1)&0xFF)<<8) | ((imm2)&0xFF) )
//@}

/**
 * @name Command List
 */
//@{
/**
 * @brief Return code for Command
 */
typedef enum
{
	HUB_MGR_CMD_RET_OK				= 0,	/// OK
	HUB_MGR_CMD_RET_E_SENSOR_ID		= -1,	/// Invalid Sensor ID
	HUB_MGR_CMD_RET_E_CMD_ID		= -2,	/// Invalid CMD ID
	HUB_MGR_CMD_RET_E_PARAMETER		= -3,	/// Parameter error
	HUB_MGR_CMD_RET_E_STATUS		= -4,	/// Status error
} HUB_MGR_CMD_RET_CODE;

/**
 * @brief deactivate sensor
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x00, sen_id, 0x00, 0x00)
 *
 * @note unsigned char sen_id: sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_DEACTIVATE_SENSOR		0x00

/**
 * @brief set sensor activate
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x01, sen_id, dst, int_no)
 *
 * @note unsigned char sen_id: sensor id
 * @note unsigned char dst: (0:to HW FIFO, 1:to SW FIFO(#HUB_MGR_CMD_PULL_SENSOR_DATA)
 * @note unsigned char f_int: (0:output without interrupt signal, 1:output with interrupt signal)
 *
 * @note (dst == 0): If HW FIFO is full, you will drop some sensor data after timing when HW FIFO is full.
 * @note (dst == 1): It pushes sensor data to SW FIFO.
 * If SW FIFO was nearly full, you will get HUB_MGR's output with interrupt signal.
 * If you don't pull data from SW FIFO, It is overwritten on old data.
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_SENSOR_ACTIVATE		0x01

/**
 * @brief set sensor interval
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x02, sen_id, 0x00, 0x00)
 * @param cmd_param[0] int interval tick_num [msec]
 *
 * @note sen_id sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_SENSOR_INTERVAL		0x02

/**
 * @brief get sensor data
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x03, sen_id, 0x00, 0x00)
 *
 * @note sen_id sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_GET_SENSOR_DATA			0x03

/**
 * @brief pull unit of sensor data from SW FIFO
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x04, 0x00, 0x00, 0x00)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, >=0:remain FIFO unit
 */
#define HUB_MGR_CMD_PULL_SENSOR_DATA		0x04

/**
 * @brief setting HUB_MGR
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x05, int_no, level, 0x00)
 *
 * @note char int_no: (0~n:with edge signal on specific GPIO#int_no at HUB_MGR data output, <0:without interrupt signal)
 * @note char level: 0: Low before generate edge, !0: High before genearate edge
 *
 * @note default is without signal
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_SETTING				0x05

/**
 * @brief get FIFO empty words
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x06, 0x00, 0x00, 0x00)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, >=0:remain FIFO empty words
 *
 * @note 1 unit = <header>(1word) + <timestamp>(1 word) + (word of sensor output data)
 */
#define HUB_MGR_CMD_GET_FIFO_EMPTY_SIZE		0x06

/**
 * @brief get HUB_MGR Version
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x07, 0, 0, 0)
 *
 * @return #HUB_MGR_VERSION
 */
#define HUB_MGR_CMD_GET_VERSION				0x07

/**
 * @brief get sensor activate
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x08, sen_id, 0, 0)
 *
 * @note sen_id sensor id
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, 0: deactive, 1: active
 */
#define HUB_MGR_CMD_GET_SENSOR_ACTIVATE		0x08

/**
 * @brief setting low power mode
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x09, usb_mode, gpio_no, hw_fifo)
 *
 * @note unsigned power_mode: (0:High speed sleep mode 1:Low speed sleep mode 2:Low speed stop mode)
 *
 * @note gpio_no: GPIO No. to notify access intension from Host CPU
 *
 * @note read_hw_fifo: (0:Not get into low power mode while HW FIFO has data 1:Get into low power mode despite of HW FIFO has data)
 *
 * @note default is without signal
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_POWER_MODE				0x09

/**
 * @brief push machine status start
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0a, 0, 0, 0)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, 0: deactive, 1: active
 */
#define HUB_MGR_CMD_PUSH_CHANGE_STATE			0x0A

/**
 * @brief send sensor status start
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0b, sen_id, 0, 0)
 *
 * @note sen_id sensor id
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, B0:Sensor Sign up , B1: Sensor Init Done ,  B2:Sensor Active ,  B3:Sensor Error
 */
#define HUB_MGR_CMD_GET_SENSOR_STATE			0x0B

/**
 * @brief send sensor action information
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0c, sen_id, 0, 0)
 *
 * @note sen_id sensor id
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE,  Sensor Status
 */
#define HUB_MGR_CMD_GET_SENSOR_OPE_STATE		0x0C

/**
 * @brief send sensor health check (cycle send)
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0d, cycle, 0, 0)
 *
 * @note cycle Transmission interval(ms)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE,  Sensor Status
 */
#define HUB_MGR_CMD_PUSH_HEALTH_STATE			0x0D

/**
 * @brief send debug data
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0e, level, 0, 0)
 *
 * @note level  Send debug Level
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE,  Debug Infomation
 */
#define HUB_MGR_CMD_PUSH_DEBUG_DATA				0x0E

/**
 * @brief check gpio state
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x0f, gpio_num, direction, out_data)
 *
 * @note level  Send debug Level
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE,  Input Data
 */
#define HUB_MGR_CMD_SET_GPIO					0x0F

/**
 * @brief set flag of interruption from frizz to HOST
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x10, imm0, imm1, 0)
 *
 * @note imm0 Event of status which latest is available, 0:OFF, 0!:ON
 *
 * @note imm1 Event of exceeding SW FIFO threshold, 0:OFF, 0!:ON
 *
 * @return hub_mgr_t:data[1]
 */
#define HUB_MGR_CMD_SET_SENSOR_INT_MASK			0x10

/**
 * @brief deactivate sensor secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x40, sen_id, 0x00, 0x00)
 *
 * @note unsigned char sen_id: sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_DEACTIVATE_SENSOR_SECONDARY			0x40

/**
 * @brief set sensor activate secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x41, sen_id, dst, int_no)
 *
 * @note unsigned char sen_id: sensor id
 * @note unsigned char dst: (0:to HW FIFO, 1:to SW FIFO(#HUB_MGR_CMD_PULL_SENSOR_DATA)
 * @note unsigned char f_int: (0:output without interrupt signal, 1:output with interrupt signal)
 *
 * @note (dst == 0): If HW FIFO is full, you will drop some sensor data after timing when HW FIFO is full.
 * @note (dst == 1): It pushes sensor data to SW FIFO.
 * If SW FIFO was nearly full, you will get HUB_MGR's output with interrupt signal.
 * If you don't pull data from SW FIFO, It is overwritten on old data.
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_SENSOR_ACTIVATE_SECONDARY		0x41

/**
 * @brief set sensor interval secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x42, sen_id, 0x00, 0x00)
 * @param cmd_param[0] int interval tick_num [msec]
 *
 * @note sen_id sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_SET_SENSOR_INTERVAL_SECONDARY		0x42

/**
 * @brief get sensor data secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x43, sen_id, 0x00, 0x00)
 *
 * @note sen_id sensor id
 *
 * @return #HUB_MGR_CMD_RET_CODE
 */
#define HUB_MGR_CMD_GET_SENSOR_DATA_SECONDARY			0x43

/**
 * @brief get FIFO empty words
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x46, 0x00, 0x00, 0x00)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, >=0:remain FIFO empty words
 *
 * @note 1 unit = <header>(1word) + <timestamp>(1 word) + (word of sensor output data)
 */
#define HUB_MGR_CMD_GET_FIFO_EMPTY_SIZE_SECONDARY		0x46

/**
 * @brief get sensor activate secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x48, sen_id, 0, 0)
 *
 * @note sen_id sensor id
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, 0: deactive, 1: active
 */
#define HUB_MGR_CMD_GET_ACTIVATE_SECONDARY				0x48

/**
 * @brief set flag of interruption from frizz to HOST for secondary
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0x50, imm0, imm1, 0)
 *
 * @note imm0 Event of status which latest is available, 0:OFF, 0!:ON
 *
 * @note imm1 Event of exceeding SW FIFO threshold, 0:OFF, 0!:ON
 *
 * @return hub_mgr_t:data[1]
 */
#define HUB_MGR_CMD_SET_SENSOR_INT_MASK_SECONDARY    0x50

/**
 * @brief get sensor version
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0xFF, 0, 0, 0)
 *
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, version
 */
#define	HUB_MGR_CMD_GET_FRIZZ_VERSION				(0xFF)
#define	HUB_MGR_CMD_GET_SENSOR_GET_FRIZZ_VERSION	HUB_MGR_CMD_GET_FRIZZ_VERSION

/**
 * @brief get sensor make date
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0xFE, 0 , 0, 0)
 *
 * @note sen_id sensor id
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, date
 */
#define	HUB_MGR_CMD_GET_SDK_VERSION					(0xFE)
#define	HUB_MGR_CMD_GET_SENSOR_GET_SDK_VERSION		HUB_MGR_CMD_GET_SDK_VERSION

/**
 * @brief get custom code
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0xFD, 0 , 0, 0)
 *
 * @return <0: #HUB_MGR_CMD_RET_CODE, version
 */
#define	HUB_MGR_CMD_GET_CUSTOMER_NAME					(0xFD)

/**
 * @brief get current timestamp
 *
 * @param cmd_code #HUB_MGR_GEN_CMD_CODE (0xFC, 0, 0, 0)
 *
 * @return current timestamp
 */
#define HUB_MGR_CMD_GET_CURRENT_TS					(0xFC)
//@}



#define DISABLING_INTERRUPT	(0)	/**< disabling gpio*/
#define ENABLING_INTERRUPT	(1)	/**< enabling gpio*/

//USE MODE
#define HIGH_SPEED_SLEEP_MODE	(0)	/**<high speed sleep mode*/
#define LOW_SPEED_SLEEP_MODE	(1)	/**<low speed sleep mode*/
#define	LOW_SPEED_STOP_MODE		(2)	/**<high speed stop mode*/

//USE Mode Command (for getting into low power mode while HW FIFO has data)
#define READ_FIFO_SIZE		(0)	/**<Not get into low power mode while HW FIFO has data*/
#define NOT_READ_FIFO_SIZE	(1)	/**<Get into low power mode despite of FIFO has data*/

/**
 * @name USB Mode Command
 */
typedef enum
{
	CMD_HIGH_SPEED_SLEEP_MODE	= 0,	//high speed sleep mode
	CMD_LOW_SPEED_SLEEP_MODE	= 1,	//low speed sleep mode
	CMD_LOW_SPEED_STOP_MODE		= 2,	//high speed stop mode
} USB_MODE_CMD;

#endif
