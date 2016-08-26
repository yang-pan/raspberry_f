/*!******************************************************************************
 * @file    spi.c
 * @brief   spi interface
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
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

// デバッグ用メッセージ出力
#ifdef D_DBG_PRINT_ENABLE
#define DBG_PRINT(...)	printf("%s(%d): ", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// エラー用メッセージ出力
#ifdef D_DBG_ERR_ENABLE
#define DBG_ERR(...)	fprintf(stderr, "[ERR] %s(%d): ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG_ERR(...)
#endif

#define D_SPI_CMD_READ	(0x00)	// frizz SPI通信用コマンド 読み出し
#define D_SPI_CMD_WRITE	(0x80)	// frizz SPI通信用コマンド 書き込み

#define D_MAX_TRANSFER_SIZE	(256) // 最大転送データサイズ

//#define D_PRINT_PACKET	// この定義を有効にすると送受信時のパケット内容を画面出力する

static int spi_fd = 0;
static uint8_t mode;
static uint8_t bits = 8;
static uint32_t speed = 14 * 1000 * 1000; // 14MHz
static uint16_t delay;

static pthread_mutex_t spi_mutex = PTHREAD_MUTEX_INITIALIZER ;

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
  * SPIデバイスをオープンし、使用できるようにする
  */
int spi_open( const char* serial_dev_path )
{
	int ret = 0;
	int result = D_RESULT_ERROR;

	// open済みの場合は成功として返す
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

/*!
 * SPI termination processing
 *
 * @param[in]
 * @return
 */
void spi_close( void )
{
	if( spi_fd ) {
		close( spi_fd );
	}
	spi_fd = 0;
}

/*!
 * SPI 32bit data of writing processing
 *
 * @param[in] register address
 * @param[in] writing data
 * @return 0=sucess, otherwise 0=fail
 */
int spi_write_reg_32( unsigned int reg_addr, unsigned int data )
{
	uint8_t	tx[6], rx[6];
	uint8_t* p;

	memset( tx, 0, sizeof( tx ) );
	memset( rx, 0, sizeof( rx ) );

	p = ( uint8_t* )&data;

	tx[0] = D_SPI_CMD_WRITE | ( 0xFF & ( reg_addr >> 8 ) );		// 書き込みコマンド
	tx[1] = 0xFF & reg_addr;
	tx[2] = 0xFF & *( p + 3 );
	tx[3] = 0xFF & *( p + 2 );
	tx[4] = 0xFF & *( p + 1 );
	tx[5] = 0xFF & *( p + 0 );
	return  spi_transfer( tx, rx, sizeof( tx ) );
}

/*!
 * SPI 32bit data of reading processing
 *
 * @param[in] register address
 * @param[out] reading data
 * @return 0=sucess, otherwise 0=fail
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
	tx[0] = D_SPI_CMD_READ | ( 0xFF & ( reg_addr >> 8 ) );		// 読み出しコマンド
	tx[1] = 0xFF & reg_addr;	// アドレス
	tx[2] = 0;	// data部
	tx[3] = 0;	// data部
	tx[4] = 0;	// data部
	tx[5] = 0;	// data部
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
 *  指定したレジスタに対し連続して書込む
 *  書き込みデータのバイト数は4の倍数でなければならない。
 *  正常時：D_RESULT_SUCCESSを返す
 *  失敗時：D_RESULT_ERRORを返す
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
		DBG_ERR( "write_size は4の倍数でなければなりません。\n" );
		return D_RESULT_ERROR;
	}
	if( write_size > D_MAX_TRANSFER_SIZE ) {
		DBG_ERR( "write_size は%d以下でなければなりません。\n", D_MAX_TRANSFER_SIZE );
		return D_RESULT_ERROR;
	}
	tx[0] = D_SPI_CMD_WRITE | ( 0xFF & ( reg_addr >> 8 ) );		// 読み出しコマンド
	tx[1] = 0xFF & reg_addr;	// アドレス
	memcpy( &tx[2], write_buff, write_size );
	return spi_transfer( tx, rx, write_size + 2 ); // +2はCMD分
}

/**
 *  指定したレジスタから指定バイト数分データを読み出す
 *  バイト数は4の倍数でなければならない
 *  正常時：D_RESULT_SUCCESSを返す
 *  失敗時：D_RESULT_ERRORを返す
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
		DBG_ERR( "read_size は4の倍数でなければなりません。\n" );
		return D_RESULT_ERROR;
	}
	if( read_size > D_MAX_TRANSFER_SIZE ) {
		DBG_ERR( "read_size は%d以下でなければなりません。\n", D_MAX_TRANSFER_SIZE );
		return D_RESULT_ERROR;
	}

	memset( tx, 0, sizeof( tx ) );
	tx[0] = D_SPI_CMD_READ | ( 0xFF & ( reg_addr >> 8 ) );		// 読み出しコマンド
	tx[1] = 0xFF & reg_addr;	// アドレス
	ret = spi_transfer( tx, rx, read_size + 2 ); // +2はCMD分
	if( ret != D_RESULT_SUCCESS ){
		memset( read_buff, 0, read_size );	// 0 padding
		return D_RESULT_ERROR;
	}

	memcpy( read_buff, &rx[2], read_size );
	return D_RESULT_SUCCESS;
}
