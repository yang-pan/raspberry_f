/*!******************************************************************************
 * @file    main.c
 * @brief   main thread for sensor logger
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
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

#define D_SPI_DEV_PATH "/dev/spidev0.0"			// The device file of SPI which connect with frizz 
#define D_FRIZZ_FIWMWARE_PATH	"bin/from.bin"	// Firmware of frizz

#define D_INPUT_BUFF_SIZE	(256)

static frizzCntrollerArg *frzzCtrlArg;

/*!********************************************************************
 *@brief      Initialize thread attributes
 *@par        Inner functions
 *
 *@param      attr_thread		thread attributes
 *@param      prioryty		schedule prioryty
 *
 *@retval     D_RESULT_SUCCESS			initialization successfully
 *@retval     D_RESULT_ERROR				error
 *
**********************************************************************/
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
    default:
        break;
    }

    return D_RESULT_SUCCESS;
}
#endif

/*!********************************************************************
 *@brief      Main function
 *@par        External public functions
 *
 *@param      argc		argument count
 *@param      argv		argument vector
 *
 *@retval     0			finished
 *
**********************************************************************/
int main( int argc, char **argv )
{
    pthread_t th_frizzctrl;
    pthread_attr_t attr;
    thread_event_t	ev;
    char inbuff[D_INPUT_BUFF_SIZE];

    DBG_PRINT( "--- Frizz Sample program: start ---\n" );

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
    pipe( frzzCtrlArg->thif.pipe_out );	// frizz Controller thread -> Main thread

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

    // Set frizz Controller thread priority
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
            break;
#else
            exit( EXIT_SUCCESS );
#endif
        }
    }

    DBG_PRINT( "Waiting for finishing all threads\n" );
    pthread_join( th_frizzctrl, NULL ); /*Wait until thread_func() been killed*/
    DBG_PRINT( "--- Frizz Sample program: end ---\n" );
    
    return 0;
}
