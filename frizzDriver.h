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

// frizz Host I/F Register
#define D_FRIZZ_REG_ADDR_CTRL		(0x00)
#define D_FRIZZ_REG_ADDR_VER		(0x01)
#define D_FRIZZ_REG_ADDR_MES		(0x02)
#define D_FRIZZ_REG_ADDR_MODE		(0x03)
#define D_FRIZZ_REG_ADDR_FIFO_CNR	(0x3f)
#define D_FRIZZ_REG_ADDR_FIFO		(0x40)
#define D_FRIZZ_REG_ADDR_RAM_ADDR	(0x52)
#define D_FRIZZ_REG_ADDR_RAM_DATA	(0x53)

// Value for CTRL register
#define D_FRIZZ_CTRL_RUN				(0x0000)
#define D_FRIZZ_CTRL_SYSTEM_RESET 		(0x0001)
#define D_FRIZZ_CTRL_STALL	  			(0x0002)

#define D_PACKET_TYPE_SENSOR_DATA		(0x80)
#define D_PACKET_TYPE_COMMAND			(0x81)
#define D_PACKET_TYPE_ACK				(0x82)
#define D_PACKET_TYPE_NACK				(0x83)
#define D_PACKET_TYPE_RES				(0x84)
#define D_PACKET_TYPE_BREAK_CODE		(0x8f)


#define D_PACKET_ACK	(0xFF82FF00)	// Ack Packet
#define D_PACKET_NACK	(0xFF83FF00)	// Nack Packet

#define FRIZZ_PACKET_DATA_MAX (64)
#define D_IS_SENSOR_DATA(p)	((p->header.type == 0x80) && (p->header.prefix == 0xFF))
#define D_TV2USEC(t)	(t.tv_sec * 1000000 + t.tv_usec)

typedef union {
	unsigned int	w;
	struct {
		unsigned char	num;		///< payload word num
		unsigned char	sen_id;		///< sensor ID
		unsigned char	type;		///< 0x80: SensorOutput, 0x81: Command, 0x82: MessageACK, 0x83: MessageNACK, 0x84: Response, 0x8F: BreakCode
		unsigned char	prefix;		///< 0xFF
	};
} hubhal_format_header_t;

typedef struct {
	hubhal_format_header_t header;
	unsigned int data[FRIZZ_PACKET_DATA_MAX];
} frizz_packet_t;

typedef struct {
	int curr;
	int next;
	unsigned int buff[FRIZZ_PACKET_DATA_MAX];
} fifo_queue_t;

/**
 *  Write firmware into frizz's I-ram 
 */
int frizzdrv_frizz_fw_download(const char* firmware_path );

/**
 *  Get data packet from frizz
 */
int frizzdrv_polling_data( void );

/**
 *  send packet to frizz   
 */
int frizzdrv_send_frizz_packet( frizz_packet_t * packet );

/**
 * Send command to sensor
 */
int frizzdrv_send_sensor_command( libsensors_id_e sen_id, int command, int parm_length, void *parm);

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
int frizzdrv_set_sensor_active( libsensors_id_e sen_id, int enabled, int use_fifo, int use_int );

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
int frizzdrv_set_gpio_irq( unsigned int gpio_num, int gpio_level );

/**
 *  Get frizz version number from register 
 *  return value: version number (sucess)
 *                      -1       (failed)
 */
int frizzdrv_get_frizz_version( void );

#endif // __SENSOR_BUFF_H__
