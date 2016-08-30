/*!******************************************************************************
 * @file    frizzDriver.c
 * @brief   frizz driver
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#define _LARGEFILE64_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include "common.h"
#include "serial.h"
#include "frizzDriver.h"
#include "sensor_buff.h"

// Debug message
#ifdef D_DBG_PRINT_ENABLE
#define DBG_PRINT(...)	printf("%s(%d): ", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// Err message
#ifdef D_DBG_ERR_ENABLE
#define DBG_ERR(...)	fprintf(stderr, "[ERR] %s(%d): ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG_ERR(...)
#endif



static fifo_queue_t fifo_q;
/**
 *  set frizz RESET/STALL
 */
static int frizz_hardware_stall( void )
{
	int ret;
	DBG_PRINT( "frizz RESET/STALL start\n" );
	ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_CTRL, D_FRIZZ_CTRL_SYSTEM_RESET );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing reset command to ctrl reg failed\n" );
		return D_RESULT_ERROR;
	}
	usleep( 100 * 1000 ); // 100ms
	ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_CTRL, D_FRIZZ_CTRL_STALL );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing stall command to ctrl reg failed\n" );
		return D_RESULT_ERROR;
	}
	DBG_PRINT( "frizz RESET/STALL done\n" );
	return D_RESULT_SUCCESS;
}

/**
 *  set frizz RESET/RUN
 */
static int frizz_hardware_reset( void )
{
	int ret;
	DBG_PRINT( "frizz RESET/RUN start\n" );
	ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_CTRL, D_FRIZZ_CTRL_SYSTEM_RESET );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing reset command to ctrl reg failed\n" );
		return D_RESULT_ERROR;
	}
	usleep( 100 * 1000 ); // 100ms
	ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_CTRL, D_FRIZZ_CTRL_RUN );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing run command to ctrl reg failed\n" );
		return D_RESULT_ERROR;
	}
	DBG_PRINT( "frizz RESET/RUN end\n" );
	return D_RESULT_SUCCESS;
}

/**
 *  read cnr
 *  return value: cnt (read success)         
 *                 -1 (read failed )
 */
static int frizz_get_cnr( void )
{
	unsigned int cnr;
	int ret;
	int buff_num;

	buff_num = fifo_q.next - fifo_q.curr;
	if( buff_num > 0 ) {
		return buff_num;
	}

	// get cnr 
	ret = serial_read_reg_32( D_FRIZZ_REG_ADDR_FIFO_CNR, &cnr );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "read cnr failed\n" );
		return -1;
	}
	cnr = 0xFFFF & ( cnr >> 16 );

	return cnr;
}

/**
 *  read one word from fifo
 *  return value: D_RESULT_SUCCESS
 *                D_RESULT_ERROR
 *  
 */
static int frizz_get_fifo( unsigned int *read_buff )
{
	int cnr;
	int ret;

	// Get data from buffer
	if( fifo_q.next > fifo_q.curr ) {
		*read_buff = fifo_q.buff[fifo_q.curr];
		common_changeEndian( read_buff );
		fifo_q.curr++;
		return D_RESULT_SUCCESS;
	}
	// no data in buff,get from fifo(empty fifo)
	cnr = frizz_get_cnr();
	if( cnr <= 0 ) {
		return D_RESULT_ERROR;
	}

	fifo_q.curr = fifo_q.next = 0;
	ret = serial_read_burst( D_FRIZZ_REG_ADDR_FIFO, ( unsigned char * )fifo_q.buff, cnr * 4 );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "read fifo failed\n" );
		return D_RESULT_ERROR;
	}
	fifo_q.next = cnr;
	*read_buff = fifo_q.buff[fifo_q.curr];
	common_changeEndian( read_buff );
	fifo_q.curr++;
	return D_RESULT_SUCCESS;
}

/**
 * write data to ram_data continuously
 */
static int frizz_write_ram( unsigned int ram_addr, unsigned char *write_data, unsigned int size )
{
	unsigned int write_size;
	unsigned char *pwrite = write_data;

	// set adress 
	if( serial_write_reg_32( D_FRIZZ_REG_ADDR_RAM_ADDR, ram_addr ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing ram_addr reg failed\n" );
		return D_RESULT_ERROR;
	}

	// write 256byte each time
	while( size != 0 ) {
		if( size >= 256 ) {
			write_size = 256;
		} else {
			write_size = size;
		}
		if( serial_write_burst( D_FRIZZ_REG_ADDR_RAM_DATA, pwrite, write_size ) != D_RESULT_SUCCESS ) {
			DBG_ERR( "writing ram_data reg failed\n" );
			return D_RESULT_ERROR;
		}
		pwrite += write_size;
		size -= write_size;
	}
	return D_RESULT_SUCCESS;
}

/**
 *  read data from ram_data continuously
 */
static int frizz_read_ram( unsigned int ram_addr, unsigned char *read_buff, unsigned int size )
{
	unsigned int read_size;
	unsigned char *pread = read_buff;

	// read 256byte 
	while( size != 0 ) {
		if( size >= 256 ) {
			read_size = 256;
		} else {
			read_size = size;
		}
		// set address
		serial_write_reg_32( D_FRIZZ_REG_ADDR_RAM_ADDR, ram_addr );

		serial_read_burst( D_FRIZZ_REG_ADDR_RAM_DATA, pread, read_size );
		ram_addr += read_size / 4;
		pread += read_size;
		size -= read_size;
	}
	return D_RESULT_SUCCESS;
}

/**
 * Get packet from frizz
 * return value: D_RESULT_SUCCESS
 *               D_RESULT_ERROR
 */
static int get_packet( frizz_packet_t *packet )
{
	//#define D_PRINT_RECEIVE_PACKET
	unsigned int rcv;
	int i;
	// Receive header of packet
	if( frizz_get_fifo( &rcv ) != D_RESULT_SUCCESS ) {
		return D_RESULT_ERROR;
	}
	packet->header.w = rcv;
#ifdef D_PRINT_RECEIVE_PACKET
	DBG_PRINT( "header: num:%d, sen_id:0x%02x, type:0x%02x, prefix:0x%02x \n",
	           packet->header.num, packet->header.sen_id, packet->header.type, packet->header.prefix );
#endif
	for( i = 0; i < packet->header.num; i++ ) {
		if( frizz_get_fifo( &rcv ) != D_RESULT_SUCCESS ) {
			return D_RESULT_ERROR;
		}
		packet->data[i] = rcv;
#ifdef D_PRINT_RECEIVE_PACKET
		DBG_PRINT( "data[%2d]: 0x%08x\n", i, packet->data[i] );
#endif
	}
	return D_RESULT_SUCCESS;
}

/**
 *  analyze the package get from frizz
 */
static int analyze_packet( frizz_packet_t *packet )
{
	int i,output_flag;
	output_flag = 0;
	
    if( D_IS_SENSOR_DATA( packet ) ) {
    	DBG_PRINT( "Receive Data! " );
    	output_flag++;
	}
	else if( packet->header.w == 0xFF84FF02 ) { 
		DBG_PRINT( "Receive HUB_MANAGER Response! " );
		output_flag++;
	}
	else if( (packet->header.w && 0xFFFF0000 ) == 0xFF840000 ){
		DBG_PRINT( "Receive sensor Response! " );
		output_flag++;
	}
	
	if( output_flag !=0 ) {
		unsigned char *p;
		DBG_PRINT( "header.num:%d, sen_id:0x%02x, type:0x%02x, prefix:0x%02x data: ",
		packet->header.num, packet->header.sen_id, packet->header.type, packet->header.prefix );
		for( i = 0; i < packet->header.num; i++ ) {
			p = ( unsigned char* )&packet->data[i];
			printf( "%02X %02X %02X %02X ", *( p + 0 ), *( p + 1 ), *( p + 2 ), *( p + 3 ) );
		}
		printf( "\n" );
	}
	
	return D_RESULT_SUCCESS;
}

/**
 * wait ACK.
 * output the package recevied during the waitting time to screen
 * Throw timeout each 10ms if not get package
 */
static int wait_ack( int timeout_ms )
{
	int tmp, ret;
	frizz_packet_t rcv_packet;

	// 10m
	tmp = timeout_ms % 10;
	timeout_ms -= tmp;

	while( timeout_ms > 0 ) {
		if( frizz_get_cnr() > 0 ) {
			ret = get_packet( &rcv_packet );
			if( ret != D_RESULT_SUCCESS ) {
				DBG_ERR( "packet receive failed\n" );
				return D_RESULT_ERROR;
			}
			if( rcv_packet.header.w == D_PACKET_ACK ) {
				DBG_PRINT( "Receive ACK\n" );
				return D_RESULT_SUCCESS;
			} else if( rcv_packet.header.w == D_PACKET_NACK ) {
				DBG_PRINT( "Receive NACK\n" );
				return D_RESULT_ERROR;
			} else {
				analyze_packet( &rcv_packet );
			}
		}
		usleep( 10 * 1000 ); // sleep 10ms
		timeout_ms -= 10;
	}
	return D_RESULT_ERROR;
}

/**
 * send command to frizz
 * include ack package processing(ignore Response package)
 */
static int send_packet( const frizz_packet_t *packet )
{
	int ret, i;
	// write packet header
	DBG_PRINT( "write packet->header=0x%08x\n", packet->header.w );
	ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_MES, packet->header.w );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "serial write failed: packet->header.w=0x%08x\n", packet->header.w );
		return D_RESULT_ERROR;
	}

	// Waiting for ack from frizz. Timeout is 1000ms.
	if( wait_ack( 1000 ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "Can't receive ACK\n" );
		return D_RESULT_ERROR;
	}

	// Send data of packet
	for( i = 0; i < packet->header.num; i++ ) {
		// write send_packet data
		DBG_PRINT( "write packet->data[%d]=0x%08x\n", i, packet->data[i] );
		ret = serial_write_reg_32( D_FRIZZ_REG_ADDR_MES, packet->data[i] );
		if( ret != D_RESULT_SUCCESS ) {
			DBG_ERR( "serial write failed: packet->header.w=0x%08x\n", packet->header.w );
			return D_RESULT_ERROR;
		}
		// Waiting for ack from frizz. Timeout is 1000ms.
		if( wait_ack( 1000 ) != D_RESULT_SUCCESS ) {
			DBG_ERR( "Can't receive ACK\n" );
			return D_RESULT_ERROR;
		}
	}
	return D_RESULT_SUCCESS;
}


/**
 * Download firmware to frizz
 */
static int frizz_fw_download( int frizz_fp )
{
	int ret;
	int read_file_size;
	unsigned int header;
	int modified_ram_size;
	int data_size;
	//	unsigned int cmd;
	unsigned int ram_addr;
	unsigned char *write_ram_data;
	unsigned char *read_ram_data;
	unsigned char read_data[4];
	int i;

	// initialize fifo
	fifo_q.next = 0;
	fifo_q.curr = 0;
	memset( fifo_q.buff, 0, sizeof( fifo_q.buff ) );

	// RESET/STALL
	ret = frizz_hardware_stall();
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "writing reset command to ctrl reg failed\n" );
		return D_RESULT_ERROR;
	}

	do {
		read_file_size = read( frizz_fp, read_data, 1 );
		header = read_data[0];
		if( header == 0xC9 ) {
			// get length
			read_file_size = read( frizz_fp, read_data, 3 );
			data_size = ( ( read_data[0] << 16 ) | ( read_data[1] << 8 ) | ( read_data[2] ) ) - 6;

			// get command for SPI/I2C(not use)
			read_file_size = read( frizz_fp, read_data, 2 );

			// get address to write 
			read_file_size = read( frizz_fp, read_data, 4 );
			ram_addr = ( read_data[0] << 24 ) | ( read_data[1] << 16 ) | ( read_data[2] << 8 ) | ( read_data[3] );

			// keep memory
			write_ram_data = malloc( data_size );
			read_file_size = read( frizz_fp, write_ram_data, data_size );

			modified_ram_size  = data_size + 3;
			modified_ram_size &= 0xFFFFFFFC;

			// write firmware to RAM
			DBG_PRINT( "frizz_write_ram Start(ram_addr=0x%08x, write_ram_data=%p, modified_ram_size=%d)\n",
			           ram_addr, write_ram_data, modified_ram_size );
			frizz_write_ram( ram_addr, write_ram_data, modified_ram_size );
			DBG_PRINT( "serial_write_reg_ram_data End\n" );

			// verify the data have been written
			read_ram_data = malloc( data_size );
			DBG_PRINT( "Verify Start(ram_addr=0x%08x, read_ram_data=%p, modified_ram_size=%d)\n",
			           ram_addr, read_ram_data, modified_ram_size );
			frizz_read_ram( ram_addr, read_ram_data, modified_ram_size );
			for( i = 0; i < modified_ram_size; i++ ) {
				if( write_ram_data[i] != read_ram_data[i] ) {
					DBG_ERR( "Verify NG: i=%d, write_ram_data[i](=%d) != read_ram_data[i](=%d)\n", i, write_ram_data[i], read_ram_data[i] );
					break;
				}
			}
			free( write_ram_data );
			free( read_ram_data );

			if( i == modified_ram_size ) {
				DBG_PRINT( "Verify Success. start address %x \n", ram_addr );
			} else {
				DBG_PRINT( "Verify Failed. start address %x \n", ram_addr );
				return D_RESULT_ERROR;
			}
		} else if( header == 0xED00 ) {

			//don't execute command and skip data.
			read_file_size = read( frizz_fp, read_data, 2 );
			data_size = ( ( read_data[0] << 8 ) | ( read_data[1] ) );
			read_file_size = read( frizz_fp, read_data, 2 );
			read_file_size = read( frizz_fp, read_data, 4 );
		}
	} while( read_file_size != 0 );

	// RESET/RUN
	ret = frizz_hardware_reset();
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "frizz hardware reset failed\n" );
		return D_RESULT_ERROR;
	}

	return D_RESULT_SUCCESS;
}

/**
 *  Download firmware(get from argument path) to frizz
 */
int frizzdrv_frizz_fw_download( const char * firmware_path )
{
	int fp;
	int ret, cnt;
	unsigned int rcv_num = 0;
	unsigned int fifo[4];

	DBG_PRINT( "firmware download start\n" );
	fp = open( firmware_path, O_RDONLY | O_LARGEFILE );
	if( fp == -1 ) {
		DBG_ERR( "firmware open error. errono=%d\n", errno );
		return D_RESULT_ERROR;
	}
	ret = frizz_fw_download( fp );
	close( fp );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_PRINT( "firmware download failed\n" );
		return D_RESULT_ERROR;
	}

	// Waiting for message from frizz. Timeout is 500ms.
	cnt = 5;
	while( cnt-- ) {
		usleep( 100 * 1000 ); // 100ms
		if( frizz_get_fifo( &fifo[rcv_num] ) != D_RESULT_SUCCESS ) {
			continue;
		}
		rcv_num++;
		if( rcv_num == 4 ) {
			break;
		}
	}
	if( rcv_num != 4 ) {
		DBG_ERR( "Timeout. Can't get message from frizz.\n" );
		return D_RESULT_ERROR;
	}
	// Check message from frizz
	if( ( fifo[0] == 0xFF80FF03 ) && ( fifo[2] == 0xFFFFFFFF ) && ( fifo[3] == 0x40000000 ) ) {
		DBG_PRINT( "frizz start up done\n" );
	} else {
		DBG_ERR( "1st message from frizz is NG\n" );
		DBG_ERR( " fifo [0]=0x%08x [1]=0x%08x [2]=0x%08x [3]=0x%08x\n", fifo[0], fifo[1], fifo[2], fifo[3] );
		return D_RESULT_ERROR;
	}

	return D_RESULT_SUCCESS;
}

/**
 *  Get and analyze packet   
 */
int frizzdrv_polling( void )
{
	int ret;
	frizz_packet_t rcv_packet;
	if( frizz_get_cnr() <= 0 ) {
		return D_RESULT_ERROR;
	}
	ret = get_packet( &rcv_packet );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "packet receive failed\n" );
		return D_RESULT_ERROR;
	}
	return analyze_packet( &rcv_packet );
}

/**
 *  send packet to frizz
 */
int frizzdrv_send_frizz_packet( frizz_packet_t * packet )
{
	frizz_packet_t rcv_packet;
	int ret;
	// Send Packet
	if( send_packet( packet ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "send packet failed\n" );
		return D_RESULT_ERROR;
	}

	// Waiting for Response from frizz.
	while( 1 ) {
		if( frizz_get_cnr() <= 0 ) {
			usleep( 1 );
			continue;
		}
		ret = get_packet( &rcv_packet );
		if( ret != D_RESULT_SUCCESS ) {
			DBG_ERR( "serial write failed: rcv_packet.header.w=0x%08x\n", rcv_packet.header.w );
			return D_RESULT_ERROR;
		}
		
		if( rcv_packet.data[0] == packet->data[0] ) {
			return analyze_packet( &rcv_packet );
		}
	}
	return D_RESULT_ERROR;
}

/**
 * Send command to sensor
 */
int frizzdrv_send_sensor_command( libsensors_id_e sen_id, int command, int parm_length, void *parm)
{
	frizz_packet_t packet;
	int cnt;
	
	int *p = ( int* )parm;
	
	DBG_PRINT( "sen_id:0x%02x, parm_length:%d, parm_lenth:%d\n",
	           sen_id, command, parm_length );
	
	if( parm_length > FRIZZ_PACKET_DATA_MAX ) {
		DBG_ERR( "parm_length must less than %d\n",parm_length );
		return D_RESULT_ERROR;
	}
	
	// Make packet
	packet.header.num = parm_length;
	packet.header.sen_id = sen_id;
	packet.header.type = 0x81;
	packet.header.prefix = 0xFF;
	packet.data[0] = HUB_MGR_GEN_CMD_CODE( command, sen_id, 0, 0 );
	for( cnt = 1 ; cnt < parm_length; cnt ++ ){
		packet.data[cnt] = p[cnt-1];
	}
	return frizzdrv_send_frizz_packet(&packet);
}

/**
 * Activate/ deactivate sensor
 * sen_id: id of the sensor to activate/deactivate
 * enabled: D_FRIZZ_SENSOR_DEACTIVATE: disable the sensor
 *          D_FRIZZ_SENSOR_ACTIVATE:   enable the sensor
 * use_fifo:
 *
 */
int frizzdrv_set_sensor_active( libsensors_id_e sen_id, int enabled, int use_fifo, int use_int )
{
	frizz_packet_t packet;

	DBG_PRINT( "sen_id:0x%02x, enabled:%d, use_fifo:%d, use_int:%d\n",
	           sen_id, enabled, use_fifo, use_int );

	// Make packet
	packet.header.num = 1;
	packet.header.sen_id = HUB_MGR_ID;	// HUB_MANAGER
	packet.header.type = 0x81;
	packet.header.prefix = 0xFF;
	packet.data[0] = HUB_MGR_GEN_CMD_CODE( HUB_MGR_CMD_SET_SENSOR_ACTIVATE, sen_id, ( 0xFF & use_fifo ), ( 0xFF & use_int ) );

	return frizzdrv_send_frizz_packet(&packet);
}

/**
 * Set update time interval of sensor 
 * sen_id: id of the sensor to activate/deactivate
 * use_fifo: fifo selection 
 * use_int:  IRQ setting(frizz->raspberry)   
 */
int frizzdrv_set_sensor_interval( libsensors_id_e sen_id, int interval, int use_fifo, int use_int )
{
	frizz_packet_t packet; 
	
	// Make packet
	packet.header.num = 2;
	packet.header.sen_id = HUB_MGR_ID;	
	packet.header.type = 0x81;
	packet.header.prefix = 0xFF;
	packet.data[0] = HUB_MGR_GEN_CMD_CODE( HUB_MGR_CMD_SET_SENSOR_INTERVAL, sen_id, ( 0xFF & use_fifo ), ( 0xFF & use_int ) );
	packet.data[1] = interval;
	
	return frizzdrv_send_frizz_packet(&packet);
}

/**
 * Activate the GPIO IRQ function of frizz (IRQ: frizz -> raspberry)
 * gpio_num:   gpio number(0~3)
 * gpio_level: 0: Active High, !0: Avtive Low
 *
 */
int frizzdrv_set_gpio_irq( unsigned int gpio_num, int gpio_level )
{
	frizz_packet_t packet;

	DBG_PRINT( "set setting: gpio_num:%d, gpio_level:%d\n", gpio_num, gpio_level );

	if( gpio_num > 3 ) {
		DBG_ERR( "gpio_num must be 0-3\n" );
		return D_RESULT_ERROR;
	}

	// Make packet
	packet.header.num = 1;
	packet.header.sen_id = HUB_MGR_ID;	// HUB_MANAGER
	packet.header.type = 0x81;
	packet.header.prefix = 0xFF;
	packet.data[0] = HUB_MGR_GEN_CMD_CODE( HUB_MGR_CMD_SET_SETTING, gpio_num, gpio_level, 0 );

	return frizzdrv_send_frizz_packet(&packet);
}

/**
 *  Get frizz version number from register 
 *  return value: version number (success)
 *                D_RESULT_ERROR (failed)
 */
int frizzdrv_get_frizz_version( void )
{
	int ret;
	unsigned int read_data;

	read_data = 0;
	ret = serial_read_reg_32( D_FRIZZ_REG_ADDR_VER, &read_data );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "read ver register failed\n" );
		return D_RESULT_ERROR;
	}
	return read_data;
}
