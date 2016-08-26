/*!******************************************************************************
 * @file    serial.c
 * @brief   serial interface
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include "serial.h"

#define CONFIG_FRIZZ_SPI
//#define CONFIG_FRIZZ_I2C // 現在は未実装

#ifdef CONFIG_FRIZZ_SPI
#include "spi.h"
#elif defined CONFIG_FRIZZ_I2C
//#include "i2c.h" // 現在は未実装
#endif

void prepare_command( unsigned char command, unsigned int reg_addr, unsigned char *tx_buff )
{
	tx_buff[0] = command;
	tx_buff[1] = ( unsigned char )reg_addr;
}

void prepare_data( unsigned int data, unsigned char *tx_buff )
{
	tx_buff[2] = ( data >> 24 ) & 0xff;
	tx_buff[3] = ( data >> 16 ) & 0xff;
	tx_buff[4] = ( data >>  8 ) & 0xff;
	tx_buff[5] = ( data >>  0 ) & 0xff;
}

int serial_open(const char* serial_dev_path)
{

#ifdef CONFIG_FRIZZ_SPI
	return spi_open( serial_dev_path );
#elif defined CONFIG_FRIZZ_I2C
	return i2c_open( serial_dev_path );
#endif
}

void serial_close( void )
{

#ifdef CONFIG_FRIZZ_SPI
	spi_close();
#elif defined CONFIG_FRIZZ_I2C
	i2c_close();
#endif

}

/**
 * 指定したレジスタに値を書込む
 */
int serial_write_reg_32( unsigned int reg_addr, unsigned int data )
{
#ifdef CONFIG_FRIZZ_SPI
	return  spi_write_reg_32( reg_addr, data );
#elif defined CONFIG_FRIZZ_I2C
	return  i2c_write_reg_32( reg_addr, data );
#endif
}

/**
 * 指定したレジスタの値を読み出す
 */
int serial_read_reg_32( unsigned int reg_addr, unsigned int *data )
{
#ifdef CONFIG_FRIZZ_SPI
	return spi_read_reg_32( reg_addr, data );
#elif defined CONFIG_FRIZZ_I2C
	return i2c_read_reg_32( reg_addr, data );
#endif
}

/**
 *  指定したレジスタに対し連続して書込む
 *  書き込みデータのバイト数は4の倍数でなければならない
 */
int serial_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size )
{
#ifdef CONFIG_FRIZZ_SPI
	return spi_write_burst( reg_addr, write_buff, write_size );
#elif defined CONFIG_FRIZZ_I2C
	return i2c_write_burst( reg_addr, write_buff, write_size );
#endif
}

/**
 *  指定したレジスタから指定バイト数分データを読み出す
 *  バイト数は4の倍数でなければならない
 */
int serial_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size )
{
#ifdef CONFIG_FRIZZ_SPI
	return spi_read_burst( reg_addr, read_buff, read_size );
#elif defined CONFIG_FRIZZ_I2C
	return i2c_read_burst( reg_addr, read_buff, read_size );
#endif
}
