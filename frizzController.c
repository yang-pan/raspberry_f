/*!******************************************************************************
 * @file    frizzController.c
 * @brief   thread for frizz control
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
#include <wiringPi.h>
#include <errno.h>

#include "common.h"
#include "sensor_buff.h"
#include "frizzController.h"
#include "frizzDriver.h"
#include "serial.h"

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

#ifdef D_USE_GPIO_IRQ
#define D_RPI_GPIO_IRQ_PIN	(17) // GPIO IRQ pin number
static int pipe_gpio_irq[2];	// pipe for GPIO IRQ
#endif

static thread_if_t *pIfToMain;	// interface to communicate with main thread

/**
 *  GPIO IRQ driver
 *  triggered by frizz for processing sensor data 
 */
#ifdef D_USE_GPIO_IRQ
static void gpio_irq_handler( void )
{
	thread_event_t ev;
	ev.id = EVENT_FRIZZCTRL_GPIO_IRQ;
	ev.data = 0;
	write( pipe_gpio_irq[D_PIPE_W], &ev, sizeof( ev ) );
}
#endif

/**
  * Initialize frizz Cotroller 
  */
static int init_frizz_controller( frizzCntrollerArg *arg )
{
	int ver, ret;

	pIfToMain = &arg->thif;
	DBG_PRINT( "SPI device         : %s\n", arg->spi_dev_path );
	DBG_PRINT( "frizz firmware path: %s\n", arg->frizz_firmware_path );

	// initialize sensor buffer
	ret = senbuff_init();
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "sensor buffer initialize failed\n" );
		return D_RESULT_ERROR;
	}

	// initialize SPI
	ret = serial_open( arg->spi_dev_path );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "serial open failed\n" );
		return D_RESULT_ERROR;
	}

	// Get frizz's version info
	ver = frizzdrv_read_ver_reg();
	DBG_PRINT( "frizz version      : 0x%08x\n", ver );
	if( ver != D_FRIZZ_CHIPID ) {
		DBG_ERR( "reading frizz ver register failed\n" );
		return D_RESULT_ERROR;
	}

	// initialize frizz
	ret = frizzdrv_frizz_fw_download( arg->frizz_firmware_path );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "frizz firmware download failed\n" );
		return D_RESULT_ERROR;
	}

#ifdef D_USE_GPIO_IRQ
	// initialize the pipe for 
	if( pipe( pipe_gpio_irq ) != 0 ) {
		DBG_ERR( "pipe error. errno=%d. %s\n", errno, strerror( errno ) );
		return D_RESULT_ERROR;
	}

	// GPIO IRQ setting(GPIO 1, Active Low) -- raspberry side 
	wiringPiSetupSys();
	wiringPiISR( D_RPI_GPIO_IRQ_PIN, INT_EDGE_FALLING, gpio_irq_handler );

	// GPIO IRQ setting(GPIO 1, Active Low) -- frizz side
	ret = frizzdrv_set_setting( D_FRIZZ_GPIO_INT_NUM_1, D_FRIZZ_INT_ACTIVE_LOW );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "frizz GPIO setting failed\n" );
		return D_RESULT_ERROR;
	}
#endif

	return D_RESULT_SUCCESS;
}

/**
  * Send event to main
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
 * main function of frizz control thread
 */
void *frizzctrl_main( void *arg )
{
	int ret;
	thread_event_t ev;
	DBG_PRINT( "frizz controller thread: start\n" );

	//-------------------------------------------------------------------------
	// check argument 
	if( arg == NULL ) {
		DBG_PRINT( "arg is NULL. frizz controller thread: end\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// initialize
	if( init_frizz_controller( ( frizzCntrollerArg* ) arg ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "init_frizz_controller error\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// send EVENT_INITIALIZE_DONE to main thread  
	ev.id = EVENT_INITIALIZE_DONE;
	ev.data = 0;
	ret = sendEvent_toMain( &ev );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "sending initialize done event to main failed\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// Activate sensors
	// (accrometer)
	ret = frizzdrv_activate( SENSOR_ID_ACCEL_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate acc failed\n" );
		exit( EXIT_FAILURE );
	}

	// gyro
	ret = frizzdrv_activate( SENSOR_ID_GYRO_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate gyro failed\n" );
		exit( EXIT_FAILURE );
	}

	// compress
	ret = frizzdrv_activate( SENSOR_ID_MAGNET_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate ecompass failed\n" );
		exit( EXIT_FAILURE );
	}

	// pressure
	ret = frizzdrv_activate( SENSOR_ID_PRESSURE_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate pressure failed\n" );
		exit( EXIT_FAILURE );
	}

#ifdef D_USE_GPIO_IRQ
	struct pollfd fds[2];

	fds[0].fd = pipe_gpio_irq[D_PIPE_R];
	fds[0].events = POLLIN;
	fds[0].revents = 0;

	fds[1].fd = pIfToMain->pipe_in[D_PIPE_R];
	fds[1].events = POLLIN;
	fds[1].revents = 0;

	//message loop
	while( 1 ) {
		poll( fds , ARRAY_SIZE( fds ), -1 );

		// Message from GPIO IRQ
		if( fds[0].revents & POLLIN ) {
			do {
				if( read( fds[0].fd, &ev, sizeof( ev ) ) < 0 ) {
					break;
				}
				fds[0].revents = 0;
				poll( &fds[0] , 1, 0 );
			} while( fds[0].revents & POLLIN );

			// read sensor data if GPIO IRQ happend
			while( frizzdrv_receive_packet() == D_RESULT_SUCCESS );
			fds[0].revents = 0;
		}
		// Message from Main thread
		if( fds[1].revents & POLLIN ) {
			// 
			break;
		}
	}
#else
	while( 1 )
	{
		frizzdrv_receive_packet();
	}
#endif

	// 
	if( arg ) {
		free( arg );
	}
	DBG_PRINT( "frizz controller thread: end\n" );

	return 0;
}

