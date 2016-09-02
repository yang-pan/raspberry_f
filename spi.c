/*!******************************************************************************
 * @file    spi.c
 * @brief   spi interface
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pthread.h>

#include "common.h"

// command identifier (for frizz SPI)
#define D_SPI_CMD_READ	(0x00)	// read
#define D_SPI_CMD_WRITE	(0x80)	// write

#define D_MAX_TRANSFER_SIZE	(256)	// maximum size

//#define D_PRINT_PACKET	// print packge content

static int spi_fd = 0;
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 14 * 1000 * 1000; // 14MHz
static uint16_t delay;

static pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER;

static int spi_transfer( unsigned char *tx_buff, unsigned char *rx_buff, int buff_size )
{
    int ret;
    int result = D_RESULT_ERROR;
    //	DBG_PRINT( "tx_buff=%p, rx_buff=%p, buff_size=%d\n", tx_buff, rx_buff, buff_size );

#ifdef D_PRINT_PACKET
    int i;
    for( i = 0; i < buff_size; i++ ) {
        if( i % 8 == 0 ) {
            if( i != 0 ) {
                printf( "\n" );
            }
            printf( "tx[%02d]: ", i );
        }
        printf( "%02x ", tx_buff[i] );
    }
    printf( "\n" );
#endif

    // mutex lock
    ret = pthread_mutex_lock( &spi_mutex );
    if ( ret ) {
        DBG_ERR( "Can't lock mutex. ret = %d", ret );
        return D_RESULT_ERROR;
    }

    struct spi_ioc_transfer tr = {
        .tx_buf = ( unsigned long )tx_buff,
        .rx_buf = ( unsigned long )rx_buff,
        .len = buff_size,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
    };
    ret = ioctl( spi_fd, SPI_IOC_MESSAGE( 1 ), &tr );
    if ( ret < 1 ) {
        DBG_ERR( "can't send spi message\n" );
        goto end;
    }
    result = D_RESULT_SUCCESS;


#ifdef D_PRINT_PACKET
    for( i = 0; i < buff_size; i++ ) {
        if( i % 8 == 0 ) {
            if( i != 0 ) {
                printf( "\n" );
            }
            printf( "rx[%02d]: ", i );
        }
        printf( "%02x ", rx_buff[i] );
    }
    printf( "\n" );
#endif

end:
    //mutex_unlock;
    ret = pthread_mutex_unlock( &spi_mutex );
    if ( ret ) {
        DBG_ERR( "Can't unlock mutex. ret = %d", ret );
        return D_RESULT_ERROR;
    }
    return result;
}

/**
 * Initialize SPI
 */
int spi_open( const char* serial_dev_path )
{
    int ret = 0;
    int result = D_RESULT_ERROR;

    // Return immidiatly if already opened
    if( spi_fd != 0 ) {
        return D_RESULT_SUCCESS;
    }

    // mutex lock
    ret = pthread_mutex_lock( &spi_mutex );
    if ( ret ) {
        DBG_ERR( "Can't lock mutex. ret = %d", ret );
        return D_RESULT_ERROR;
    }

    spi_fd = open( serial_dev_path, O_RDWR );
    if ( spi_fd < 0 ) {
        DBG_ERR( "can't open device" );
        goto end;
    }

    /*
     * spi mode
     */
    // Configure mode
    mode = SPI_MODE_3;
    ret = ioctl( spi_fd, SPI_IOC_WR_MODE, &mode );
    if ( ret == -1 ) {
        DBG_ERR( "can't set spi mode" );
        goto end;
    }

    // Read mode for Verify
    ret = ioctl( spi_fd, SPI_IOC_RD_MODE, &mode );
    if ( ret == -1 ) {
        DBG_ERR( "can't get spi mode" );
        goto end;
    }

    /*
     * bits per word
     */
    ret = ioctl( spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits );
    if ( ret == -1 ) {
        DBG_ERR( "can't set bits per word" );
        goto end;
    }

    ret = ioctl( spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits );
    if ( ret == -1 ) {
        DBG_ERR( "can't get bits per word" );
        goto end;
    }

    /*
     * max speed hz
     */
    ret = ioctl( spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed );
    if ( ret == -1 ) {
        DBG_ERR( "can't set max speed hz" );
        goto end;
    }

    ret = ioctl( spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed );
    if ( ret == -1 ) {
        DBG_ERR( "can't get max speed hz" );
        goto end;
    }

    result = D_RESULT_SUCCESS;

end:
    //mutex_unlock;
    ret = pthread_mutex_unlock( &spi_mutex );
    if ( ret ) {
        DBG_ERR( "Can't unlock mutex. ret = %d", ret );
        return D_RESULT_ERROR;
    }

    //	DBG_PRINT( "spi mode: %d\n", mode );
    //	DBG_PRINT( "bits per word: %d\n", bits );
    //	DBG_PRINT( "max speed: %d Hz (%d KHz)\n", speed, speed / 1000 );

    return result;
}

/**
 * Release SPI
 */
void spi_close( void )
{
    if( spi_fd ) {
        close( spi_fd );
    }
    spi_fd = 0;
}

/**
 * Write data to register by spi
 */
int spi_write_reg_32( unsigned int reg_addr, unsigned int data )
{
    uint8_t	tx[6], rx[6];
    uint8_t* p;

    memset( tx, 0, sizeof( tx ) );
    memset( rx, 0, sizeof( rx ) );

    p = ( uint8_t* )&data;

    tx[0] = D_SPI_CMD_WRITE | ( 0xFF & ( reg_addr >> 8 ) );	// command for writting
    tx[1] = 0xFF & reg_addr;
    tx[2] = 0xFF & *( p + 3 );
    tx[3] = 0xFF & *( p + 2 );
    tx[4] = 0xFF & *( p + 1 );
    tx[5] = 0xFF & *( p + 0 );
    return  spi_transfer( tx, rx, sizeof( tx ) );
}

/**
 * read data from register by spi
 */
int spi_read_reg_32( unsigned int reg_addr, unsigned int *data )
{
    uint8_t	tx[6], rx[6];
    uint8_t* p;
    int ret;

    if( data == NULL ) {
        DBG_ERR( "data is NULL\n" );
        return D_RESULT_ERROR;
    }

    memset( tx, 0, sizeof( tx ) );
    memset( rx, 0, sizeof( rx ) );
    tx[0] = D_SPI_CMD_READ | ( 0xFF & ( reg_addr >> 8 ) );	// command for reading
    tx[1] = 0xFF & reg_addr;	// address
    tx[2] = 0;	// data part
    tx[3] = 0;	// data part
    tx[4] = 0;	// data part
    tx[5] = 0;	// data part
    ret = spi_transfer( tx, rx, sizeof( tx ) );
    if( ret != D_RESULT_SUCCESS ) {
        *data = 0;
        return D_RESULT_ERROR;
    }

    p = ( uint8_t* )data;
    *( p + 0 ) = rx[5];
    *( p + 1 ) = rx[4];
    *( p + 2 ) = rx[3];
    *( p + 3 ) = rx[2];

    return D_RESULT_SUCCESS;
}

/**
 * Write data to register continuously by spi
 */
int spi_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size )
{
    uint8_t tx[D_MAX_TRANSFER_SIZE + 2];
    uint8_t rx[D_MAX_TRANSFER_SIZE + 2];

    if( write_buff == NULL ) {
        DBG_ERR( "write_buff is NULL\n" );
        return D_RESULT_ERROR;
    }
    if( write_size % 4 != 0 ) {
        DBG_ERR( "write_size should be interger multipe of 4 bytes\n" );
        return D_RESULT_ERROR;
    }
    if( write_size > D_MAX_TRANSFER_SIZE ) {
        DBG_ERR( "write_size should less than %d\n", D_MAX_TRANSFER_SIZE );
        return D_RESULT_ERROR;
    }
    tx[0] = D_SPI_CMD_WRITE | ( 0xFF & ( reg_addr >> 8 ) );		// command for writting
    tx[1] = 0xFF & reg_addr;
    memcpy( &tx[2], write_buff, write_size );
    return spi_transfer( tx, rx, write_size + 2 );
}

/**
 * Read data from register continuously by spi
 */
int spi_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size )
{
    uint8_t tx[D_MAX_TRANSFER_SIZE + 2];
    uint8_t rx[D_MAX_TRANSFER_SIZE + 2];
    int ret;

    if( read_buff == NULL ) {
        DBG_ERR( "read_buff is NULL\n" );
        return D_RESULT_ERROR;
    }
    if( read_size % 4 != 0 ) {
        DBG_ERR( "read_size should be interger multipe of 4 bytes\n" );
        return D_RESULT_ERROR;
    }
    if( read_size > D_MAX_TRANSFER_SIZE ) {
        DBG_ERR( "read_size should less than %d\n", D_MAX_TRANSFER_SIZE );
        return D_RESULT_ERROR;
    }

    memset( tx, 0, sizeof( tx ) );
    tx[0] = D_SPI_CMD_READ | ( 0xFF & ( reg_addr >> 8 ) );	// command for reading
    tx[1] = 0xFF & reg_addr;	// address
    ret = spi_transfer( tx, rx, read_size + 2 );	// CMD size = 2
    if( ret != D_RESULT_SUCCESS ) {
        memset( read_buff, 0, read_size );	// 0 padding
        return D_RESULT_ERROR;
    }

    memcpy( read_buff, &rx[2], read_size );
    return D_RESULT_SUCCESS;
}
