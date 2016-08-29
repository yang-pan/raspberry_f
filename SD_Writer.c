/*!******************************************************************************
 * @file    SD_Writer.c
 * @brief   thread for write log data on SD card
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <poll.h>

#include "common.h"
#include "frizzController.h"
#include "sensor_buff.h"
#include "SD_Writer.h"
#include "libsensors_id.h"
#include "bmd101_sensor_writer.h"
#include "normal_sensor_writer.h"

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


enum log_chanel_e {
    CHANNEL_BMD101 = 0,
    CHANNEL_ACCELEROMETER,
    CHANNEL_GYRO,
    CHANNEL_ECOMPASS,
    CHANNEL_PRESSURE,
    CHANNEL_UNDEFINED
};

static thread_if_t *pIfToMain;	// main thread's I/F
static FILE* fp;
static int pipe_with_api[2];

/**
  * SD Writer initialize
  */
static int init_sd_writer( sdWriterArg *arg )
{
	pIfToMain = &arg->thif;
	DBG_PRINT( "log file path: %s\n", arg->log_file_path );
	fp = fopen( arg->log_file_path, "wb" );
	if( fp == NULL ) {
		DBG_PRINT( "log file can not open.\n" );
		return D_RESULT_ERROR;
	}
	pipe( pipe_with_api );
	return D_RESULT_SUCCESS;
}

/**
  * Send event to main thread
  */
static int sendEvent_toMain( thread_event_t* ev )
{
	if( write( pIfToMain->pipe_out[D_PIPE_W], ev, sizeof( thread_event_t ) ) <= 0 ) {
		DBG_ERR( "write error\n" );
		return D_RESULT_ERROR;
	}
	return D_RESULT_SUCCESS;
}

/**
  * Write data to SD card
  */
int writeDataToTheMedia( int idx )
{
	int cnt;
	cnt = write( pipe_with_api[D_PIPE_W], &idx, sizeof( idx ) );
	if( cnt != sizeof(idx) ) {
		DBG_ERR( "write error. cnt=%d\n", cnt );
		return D_RESULT_ERROR;
	}

	//	DBG_PRINT( "write success (idx = %d)\n", idx );
	return D_RESULT_SUCCESS;
}


/**
  * Receive data index from API
  */
static int recvData( sensor_data_t** received_data )
{
	int data_idx;
	if( read( pipe_with_api[D_PIPE_R], &data_idx, sizeof( data_idx ) ) < 0 ) {
		DBG_ERR( "recv error\n" );
		return -1;
	}

	if( senbuff_getBuffAddr( data_idx, received_data ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "get buffer address error\n" );
		return -1;
	}
	//	DBG_PRINT( "success recvData\n" );

	return data_idx;
}

/**
  * Convert sensor id to channel 
  */
static unsigned char getChannel( sensor_data_t* recv_data )
{
	int code;
	unsigned char channel;
	//	unsigned char* sensor_id = ((unsigned char*)(&code)) + 1;
	unsigned char sensor_id;
	code = recv_data->code;
	sensor_id = 0xFF & ( code >> 8 );

	switch( sensor_id ) {
	case SENSOR_ID_ECG_RAW:
		channel = CHANNEL_BMD101;
		break;
	case SENSOR_ID_ACCEL_RAW:
		channel = CHANNEL_ACCELEROMETER;
		break;
	case SENSOR_ID_GYRO_RAW:
		channel = CHANNEL_GYRO;
		break;
	case SENSOR_ID_MAGNET_RAW:
		channel = CHANNEL_ECOMPASS;
		break;
	case SENSOR_ID_PRESSURE_RAW:
		channel = CHANNEL_PRESSURE;
		break;
	default:
		channel = CHANNEL_UNDEFINED;
		break;
	}
	return channel;
}

void saveTheSensorData()
{
	const unsigned char kSync[] = {0xAA, 0xAA};
	sensor_data_t* recv_data;
	int idx;
	unsigned char channel;
	idx = recvData( &recv_data );
	// err check 
	if( idx < 0 ) {
		DBG_ERR( "read error\n" );
		return;
	}
	channel = getChannel( recv_data );
	switch( channel ) {
	case CHANNEL_ACCELEROMETER:
	case CHANNEL_ECOMPASS:
	case CHANNEL_PRESSURE:
	case CHANNEL_GYRO:
		// write sync
		ucharWriter( kSync, sizeof(kSync), fp );
		// write channel
		ucharWriter( &channel, 1, fp );
		normalSensorWriter( recv_data, fp );
		//		DBG_PRINT( "ch:0x%x wrote\n", channel );
		break;
	case CHANNEL_BMD101:
		//write sync
		ucharWriter( kSync, sizeof(kSync), fp );
		//write channel
		ucharWriter( &channel, 1, fp );
		BMD101SensorWriter( recv_data, fp );
		//		DBG_PRINT( "ch:0x%x wrote\n", channel );
		break;
	case CHANNEL_UNDEFINED:
	default:
		break;
	}
	fflush( fp );
	senbuff_free( idx );
}

void *sdWriter_main( void *arg )
{
	thread_event_t ev;
	DBG_PRINT( "SD Writer thread: start\n" );
	struct pollfd fds[2];

	//-------------------------------------------------------------------------
	// check argument
	if( arg == NULL ) {
		DBG_PRINT( "arg is NULL. SD Writer thread: end\n" );
		return 0;
	}

	//-------------------------------------------------------------------------
	// initialize 
	if( init_sd_writer( ( sdWriterArg* ) arg ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "init_sd_writer error\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// send EVENT_INITIALIZE_DONE to main thread 
	ev.id = EVENT_INITIALIZE_DONE;
	ev.data = 0;
	sendEvent_toMain( &ev );

	DBG_PRINT( "SD Writer thread: end\n" );

	fds[0].fd = pipe_with_api[D_PIPE_R];
	fds[0].events = POLLIN;
	fds[1].fd = pIfToMain->pipe_in[D_PIPE_R];
	fds[1].events = POLLIN;
	// message loop
	while( 1 ) {
		fds[0].revents = 0;
		fds[1].revents = 0;
		poll( fds , 2, -1 );
		if( fds[0].revents & POLLIN ) {
			saveTheSensorData();
		}
		if( fds[1].revents & POLLIN ) {
			break;
		}
	}

	// release resource 
	if( arg ) {
		free( arg );
	}
	fclose( fp );
	DBG_PRINT( "SD Writer thread: end\n" );

	return 0;
}
