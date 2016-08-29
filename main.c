/*!******************************************************************************
 * @file    main.c
 * @brief   main thread for sensor logger
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
#include <errno.h>

#include "common.h"
#include "frizzController.h"
#include "SD_Writer.h"

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

#define D_SPI_DEV_PATH "/dev/spidev0.0"			// The device file of SPI which connect with frizz 
#define D_FRIZZ_FIWMWARE_PATH	"bin/from.bin"	// Firmware of frizz

#define D_INPUT_BUFF_SIZE	(256)

static frizzCntrollerArg *frzzCtrlArg;
static sdWriterArg *sdArg;

//static int init_ThreadAttr( pthread_attr_t *attr_thread, size_t stacksize, int prioryty )
static int init_ThreadAttr( pthread_attr_t *attr_thread, int prioryty )
{
	int		ret;
	size_t	tmp_stacksize;
	int		policy;
	struct	sched_param param;

	// Initialize thread
	ret = pthread_attr_init( attr_thread );
	if( ret != 0 ) {
		DBG_ERR( "Init thread ret=%x\n", ret );
		return D_RESULT_ERROR;
	}

	// Check stack size
	ret = pthread_attr_getstacksize( attr_thread, &tmp_stacksize );
	if( ret != 0 ) {
		pthread_attr_destroy( attr_thread );
		DBG_ERR( "Get thread stack size failed! ret=%x\n", ret );
		return D_RESULT_ERROR;
	}

	policy = SCHED_FIFO;
	ret = pthread_attr_setschedpolicy( attr_thread, policy );
	if( ret != 0 ) {
		pthread_attr_destroy( attr_thread );
		DBG_ERR( "Set thread policy failed! ret=%x\n", ret );
		return D_RESULT_ERROR;
	}

	memset( &param, 0x00, sizeof( param ) );

	// Set priority
	ret = pthread_attr_getschedparam( attr_thread, &param );
	if( ret != 0 ) {
		pthread_attr_destroy( attr_thread );
		DBG_ERR( "Get scheparam failed! ret = %d(err:%d)\n", ret, errno );
		return D_RESULT_ERROR;
	}
	param.sched_priority = prioryty;
	ret = pthread_attr_setschedparam( attr_thread, &param );
	if( ret != 0 ) {
		pthread_attr_destroy( attr_thread );
		DBG_ERR( "Set thread prioryty filed! ret = %d(err:%d)\n", ret, errno );
		return D_RESULT_ERROR;
	}

	return D_RESULT_SUCCESS;
}

/**
  * Send event to main thread
  */
#ifdef D_USE_GPIO_IRQ
static int sendEvent( ThreadIdx_e thidx, thread_event_t* ev )
{
	switch( thidx ) {
	case THREAD_FRIZZ_CONTROLLER:
		if( write( frzzCtrlArg->thif.pipe_in[D_PIPE_W], ev, sizeof( thread_event_t ) ) <= 0 ) {
			DBG_ERR( "write error: %s\n", strerror( errno ) );
			return D_RESULT_ERROR;
		}
		break;
	case THREAD_SD_WRITER:
		if( write( sdArg->thif.pipe_in[D_PIPE_W], ev, sizeof( thread_event_t ) ) <= 0 ) {
			DBG_ERR( "write error: %s\n", strerror( errno ) );
			return D_RESULT_ERROR;
		}
		break;
	default:
		break;
	}

	return D_RESULT_SUCCESS;
}
#endif 

int main( int argc, char **argv )
{
	pthread_t th_frizzctrl;
	pthread_t th_sdwriter;
	pthread_attr_t attr;
	thread_event_t	ev;
	char inbuff[D_INPUT_BUFF_SIZE];

	DBG_PRINT( "--- ECG_log: start ---\n" );

	//-------------------------------------------------------------------------
	// Get log file's name from argument
	//-------------------------------------------------------------------------
	if( argc != 2 ) {
		DBG_ERR( "Usage) %s <logfile_path>\n", argv[0] );
		exit( EXIT_FAILURE );
	}
	if( strlen( argv[1] ) > ( PATH_MAX - 1 ) ) {
		DBG_ERR( "logfile path is too long\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// Start the frizz Controller Thread 
	//-------------------------------------------------------------------------
	DBG_PRINT( "Create frizz Controller thread\n" );
	// Malloc memory 
	frzzCtrlArg = malloc( sizeof( frizzCntrollerArg ) );
	if( frzzCtrlArg == NULL ) {
		DBG_ERR( "malloc error\n" );
		exit( EXIT_FAILURE );
	}
	
	pipe( frzzCtrlArg->thif.pipe_in );	// Main thread -> frizz Controller thread
	pipe( frzzCtrlArg->thif.pipe_out ); // frizz Controller thread -> Main thread

	strncpy( frzzCtrlArg->spi_dev_path, D_SPI_DEV_PATH, strlen( D_SPI_DEV_PATH ) );
	frzzCtrlArg->spi_dev_path[sizeof( frzzCtrlArg->spi_dev_path ) - 1] = 0;
	strncpy( frzzCtrlArg->frizz_firmware_path, D_FRIZZ_FIWMWARE_PATH, strlen( D_FRIZZ_FIWMWARE_PATH ) );
	frzzCtrlArg->frizz_firmware_path[sizeof( frzzCtrlArg->frizz_firmware_path ) - 1] = 0;

	// Set Attribute 
	if( init_ThreadAttr( &attr, 50 ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "setting attribute NG\n" );
		exit( EXIT_FAILURE );
	}

	// frizz Ctrl 
	if( pthread_create( &th_frizzctrl, &attr, frizzctrl_main, frzzCtrlArg ) != 0 ) {
		DBG_ERR( "pthread_create() error\n" );
		exit( EXIT_FAILURE );
	}
	
	// Set frizz Controller thread's priority
	memset( &ev, 0, sizeof( ev ) );
	if( read( frzzCtrlArg->thif.pipe_out[D_PIPE_R], &ev, sizeof( ev ) ) < 0 ) {
		DBG_ERR( "read error\n" );
		exit( EXIT_FAILURE );
	}
	if( ev.id != EVENT_INITIALIZE_DONE ) {
		DBG_ERR( "frizz Controller initialize failed\n" );
		exit( EXIT_FAILURE );
	}
	DBG_PRINT( "frizz Controller start finished\n" );

	//-------------------------------------------------------------------------
	// Start the SD Writer Thread 
	//-------------------------------------------------------------------------
	DBG_PRINT( "Create SD Writer thread\n" );
	// Malloc memory
	sdArg = malloc( sizeof( sdWriterArg ) );
	if( sdArg == NULL ) {
		DBG_ERR( "malloc error\n" );
		exit( EXIT_FAILURE );
	}
	pipe( sdArg->thif.pipe_in );
	pipe( sdArg->thif.pipe_out );

	// Be careful because have no err check here. 
	strncpy( sdArg->log_file_path, argv[1], sizeof( sdArg->log_file_path ) );
	sdArg->log_file_path[sizeof( sdArg->log_file_path ) - 1] = 0;

	if( pthread_create( &th_sdwriter, NULL, sdWriter_main, sdArg ) != 0 ) {
		DBG_ERR( "pthread_create() error\n" );
		exit( EXIT_FAILURE );
	}
	// Wait SD Writer thread start 
	memset( &ev, 0, sizeof( ev ) );
	if( read( sdArg->thif.pipe_out[D_PIPE_R], &ev, sizeof( ev ) ) < 0 ) {
		DBG_ERR( "read error\n" );
		exit( EXIT_FAILURE );
	}
	if( ev.id != EVENT_INITIALIZE_DONE ) {
		DBG_ERR( "SD Writer initialize failed\n" );
		exit( EXIT_FAILURE );
	}
	DBG_PRINT( "SD Writer start finished\n" );

	// wait 1 second to avoid console conflict with other thread 
	sleep( 1 );

	// main loop 
	while( 1 ) {
		printf( "> " );
		scanf( "%s", inbuff );
		if( strncmp( "exit", inbuff, sizeof( "exit" ) ) == 0 ) {
#ifdef D_USE_GPIO_IRQ
			memset( &ev, 0, sizeof( ev ) );
			ev.id = EVENT_FINISH_THREAD;
			sendEvent( THREAD_FRIZZ_CONTROLLER, &ev );
			sendEvent( THREAD_SD_WRITER, &ev );
			break;
#else
			exit( EXIT_SUCCESS );
#endif
		}
	}

	DBG_PRINT( "Waiting for finishing all threads\n" );
	pthread_join( th_frizzctrl, NULL ); /*Wait until thread_func() been killed*/
	pthread_join( th_sdwriter, NULL ); /*Wait undtil thread_func() been killed*/

	DBG_PRINT( "--- ECG_log: end ---\n" );
	return 0;
}
