/*!******************************************************************************
 * @file    serial.c
 * @brief   serial interface
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include "serial.h"

#define CONFIG_FRIZZ_SPI
//#define CONFIG_FRIZZ_I2C // not support now

#ifdef CONFIG_FRIZZ_SPI
#include "spi.h"
#elif defined CONFIG_FRIZZ_I2C
//#include "i2c.h" // not support now
#endif

/*!********************************************************************
 *@brief      Open serial devcie
 *@par        External public functions
 *
 *@param      serial_dev_path		path of serial device
 *
 *@retval     D_RESULT_SUCCESS		open successfully
 *@retval     D_RESULT_ERROR			open failed
 *
**********************************************************************/
int serial_open(const char* serial_dev_path)
{
#ifdef CONFIG_FRIZZ_SPI
    return spi_open( serial_dev_path );
#elif defined CONFIG_FRIZZ_I2C
    return i2c_open( serial_dev_path );
#endif
}

/*!********************************************************************
 *@brief      Close serial devcie
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     void
 *
**********************************************************************/
void serial_close( void )
{
#ifdef CONFIG_FRIZZ_SPI
    spi_close();
#elif defined CONFIG_FRIZZ_I2C
    i2c_close();
#endif
}

/*!********************************************************************
 *@brief      write register
 *@par        External public functions
 *
 *@param      reg_addr	register address
 *@param      data		pointer to the area of source data
 *
 *@retval     D_RESULT_SUCCESS		write successfully
 *@retval     D_RESULT_ERROR			write failed
 *
**********************************************************************/
int serial_write_reg_32( unsigned int reg_addr, unsigned int data )
{
#ifdef CONFIG_FRIZZ_SPI
    return  spi_write_reg_32( reg_addr, data );
#elif defined CONFIG_FRIZZ_I2C
    return  i2c_write_reg_32( reg_addr, data );
#endif
}

/*!********************************************************************
 *@brief      Read register
 *@par        External public functions
 *
 *@param      reg_addr	register address
 *@param      data		pointer to the area to store data
 *
 *@retval     D_RESULT_SUCCESS		read successfully
  @retval     D_RESULT_ERROR			error
 *
**********************************************************************/
int serial_read_reg_32( unsigned int reg_addr, unsigned int *data )
{
#ifdef CONFIG_FRIZZ_SPI
    return spi_read_reg_32( reg_addr, data );
#elif defined CONFIG_FRIZZ_I2C
    return i2c_read_reg_32( reg_addr, data );
#endif
}

/*!********************************************************************
 *@brief      write register continuously
 *             data size should be an integer multiple of 4 bytes
 *@par        External public functions
 *
 *@param      reg_addr		register address
 *@param      write_buff		pointer to the area of source data
 *@param      write_size		data size to write
 *
 *@retval     D_RESULT_SUCCESS	write successfully
  @retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int serial_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size )
{
#ifdef CONFIG_FRIZZ_SPI
    return spi_write_burst( reg_addr, write_buff, write_size );
#elif defined CONFIG_FRIZZ_I2C
    return i2c_write_burst( reg_addr, write_buff, write_size );
#endif
}

/*!********************************************************************
 *@brief      read register continuously
 *             data size should be and integer multiple of 4 bytes
 *@par        External public functions
 *
 *@param      reg_addr		register address
 *@param      read_buff		pointer to the area to store data
 *@param      read_size		data size to read
 *
 *@retval     D_RESULT_SUCCESS	read successfully
  @retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int serial_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size )
{
#ifdef CONFIG_FRIZZ_SPI
    return spi_read_burst( reg_addr, read_buff, read_size );
#elif defined CONFIG_FRIZZ_I2C
    return i2c_read_burst( reg_addr, read_buff, read_size );
#endif
}
