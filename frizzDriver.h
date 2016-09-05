/*!******************************************************************************
 * @file    frizzDriver.h
 * @brief   source for frizzDriver
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
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

#define D_FRIZZ_CHIPID					(0x00000200)	// frizz Chip ID(version)

#define D_FRIZZ_SENSOR_DEACTIVATE		(0)	// deactivate sensor.
#define D_FRIZZ_SENSOR_ACTIVATE			(1)	// activate sensor.

// frizz Host I/F Register
#define D_FRIZZ_REG_ADDR_CTRL			(0x00)
#define D_FRIZZ_REG_ADDR_VER			(0x01)
#define D_FRIZZ_REG_ADDR_MES			(0x02)
#define D_FRIZZ_REG_ADDR_MODE			(0x03)
#define D_FRIZZ_REG_ADDR_FIFO_CNR		(0x3f)
#define D_FRIZZ_REG_ADDR_FIFO			(0x40)
#define D_FRIZZ_REG_ADDR_RAM_ADDR		(0x52)
#define D_FRIZZ_REG_ADDR_RAM_DATA		(0x53)

// Value for CTRL register
#define D_FRIZZ_CTRL_RUN				(0x0000)
#define D_FRIZZ_CTRL_SYSTEM_RESET		(0x0001)
#define D_FRIZZ_CTRL_STALL				(0x0002)

// Packet indetifier
#define D_PACKET_TYPE_SENSOR_DATA		(0x80)
#define D_PACKET_TYPE_COMMAND			(0x81)
#define D_PACKET_TYPE_ACK				(0x82)
#define D_PACKET_TYPE_NACK				(0x83)
#define D_PACKET_TYPE_RES				(0x84)
#define D_PACKET_TYPE_BREAK_CODE		(0x8f)

// GPIO IRQ
#define D_FRIZZ_GPIO_INT_NUM_0			(0)	// gpio_num: use gpio number 0 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_1			(1)	// gpio_num: use gpio number 1 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_2			(2)	// gpio_num: use gpio number 2 for interrupt
#define D_FRIZZ_GPIO_INT_NUM_3			(3)	// gpio_num: use gpio number 3 for interrupt
#define D_FRIZZ_INT_ACTIVE_HIGH			(0)	// gpio_level: Low before generate edge
#define D_FRIZZ_INT_ACTIVE_LOW			(1)	// gpio_level: High before genearate edge

#define D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO			(0)	// use_fifo: sensor data is pushed to hw fifo
#define D_FRIZZ_ACTIVATE_PARAM_USE_SWFIFO			(1)	// use_fifo: sensor data is pushed to sw fifo
#define D_FRIZZ_ACTIVATE_PARAM_WITHOUT_INTERRUPT	(0)	// use_int: output without interrupt signal
#define D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT		(1)	// use_int: output with interrupt signal

#define D_PACKET_ACK	(0xFF82FF00)	// Ack Packet
#define D_PACKET_NACK	(0xFF83FF00)	// Nack Packet

#define FRIZZ_PACKET_DATA_MAX (64)
#define D_IS_SENSOR_DATA(p)	((p.header.type == 0x80) && (p.header.prefix == 0xFF))
#define D_TV2USEC(t)	(t.tv_sec * 1000000 + t.tv_usec)

/**@union  hubhal_format_header_t
 * @brief  hubhal header format
 */
typedef union {
    unsigned int	w;
    struct {
        unsigned char	num;		///< payload word num
        unsigned char	sen_id;		///< sensor ID
        unsigned char	type;		///< 0x80: SensorOutput, 0x81: Command, 0x82: MessageACK, 0x83: MessageNACK, 0x84: Response, 0x8F: BreakCode
        unsigned char	prefix;		///< 0xFF
    };
} hubhal_format_header_t;

/**@union  frizz_data_t
 * @brief  for printing sensor data
 */
typedef union {
    unsigned int	ui_val;
    float			f_val;
} frizz_data_t;

/**@struct  fifo_queue_t
 * @brief   buffer to parse data get from frizz
 */
typedef struct {
    int				curr;
    int				next;
    unsigned int	buff[FRIZZ_PACKET_DATA_MAX];
} fifo_queue_t;

/**@struct  frizz_packet_t
 * @brief   frizz packet format
 */
typedef struct {
    hubhal_format_header_t	header;
    unsigned int			data[FRIZZ_PACKET_DATA_MAX];
} frizz_packet_t;

/*!********************************************************************
 *@brief      Stall DSP core on frizz
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     D_RESULT_SUCCESS			stall successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_stall_frizz( void );

/*!********************************************************************
 *@brief      Run DSP core on frizz
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     D_RESULT_SUCCESS			run successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_run_frizz( void );

/*!********************************************************************
 *@brief      Reset DSP core on frizz
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     D_RESULT_SUCCESS			reset successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_reset_frizz( void );

/*!********************************************************************
 *@brief      Read version register on frizz
 *@par        External public functions
 *
 *@param      version		pointer to the area to store version value
 *
 *@retval     D_RESULT_SUCCESS			read successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_get_ver_reg( unsigned int*version );

/*!********************************************************************
 *@brief      Download frizz firmware to frizz RAM
 *@par        External public functions
 *
 *@param      firmware_path		file path to the frizz firmware bin file
 *
 *@retval     D_RESULT_SUCCESS			download successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_frizz_fw_download(const char* firmware_path );

/*!********************************************************************
 *@brief      Parsing fifo_q buffer to get packet comes from frizz
 *@par        External public functions
 *
 *@param      packet		pointer to the area to store data
 *
 *@retval     D_RESULT_SUCCESS			get packet successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_receive_packet( frizz_packet_t *packet );

/*!********************************************************************
 *@brief      Send packet to frizz(with ack and response processing)
 *@par        External public functions
 *
 *@param      packet		pointer to the packet data 
 *
 *@retval     D_RESULT_SUCCESS			send packet successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_send_frizz_packet( frizz_packet_t * packet );

/*!********************************************************************
 *@brief      Send command to specified sensor or HUB_MGR
 *@par        External public functions
 *
 *@param      sen_id			sensor ID to send
 *@param      command			command code to send
 *@param      param_length	size of command parameter
 *@param      param			pointer to the command parameter area
 *
 *@retval     D_RESULT_SUCCESS			send successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_send_sensor_command( libsensors_id_e sen_id, int command, int param_length, void *param);

/*!********************************************************************
 *@brief      Activate or deactivate specified sensor with specified settings
 *@par        External public functions
 *
 *@param      sen_id		sensor ID to activate or deactivate
 *@param      enabled		0:deactivate
 *                     		1:activate
 *@param      use_fifo	0:not use FIFO to store sensor data
 *                     		1:use FIFO to stre sensor data
 *@param      use_int		0:no interrupt to notify data ready
 *                     		1:use interrupt to notify data ready
 *
 *@retval     D_RESULT_SUCCESS			activate/deactivate successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_activate_sensor( libsensors_id_e sen_id, int enabled, int use_fifo, int use_int );

/*!********************************************************************
 *@brief      Set interval time to update sensor data
 *@par        External public functions
 *
 *@param      sen_id		sensor ID to set
 *@param      tick		sampling interval ( msec )
 *
 *@retval     D_RESULT_SUCCESS			set successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_set_sensor_interval( libsensors_id_e sen_id, int tick );

/*!********************************************************************
 *@brief      Set GPIO interrupt setting
 *@par        External public functions
 *
 *@param      gpio_num		the number of GPIO pin to use for interrupt
 *                      		If less than 0,disable GPIO use for interrupt
 *@param     gpio_level		0:Low( Rising edge/high active )
 *                       		1:High( Falling edge/low active )
 *
 *@retval     D_RESULT_SUCCESS			set successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_set_setting( int gpio_num, unsigned int gpio_level );

/*!********************************************************************
 *@brief      Polling fifo_q buffer to get sensor data
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     D_RESULT_SUCCESS			get sensor data successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
int frizzdrv_polling_data( void );

#endif // __SENSOR_BUFF_H__
