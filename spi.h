/*!******************************************************************************
 * @file    spi.h
 * @brief   source for spi
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/

/*!********************************************************************
 *@brief      Initialize SPI
 *@par        External public functions
 *
 *@param      serial_dev_path		path of spi device file
 *
 *@retval     D_RESULT_SUCCESS	Initialize successfully
 *@retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int spi_open( const char *spi_dev_path );

/*!********************************************************************
 *@brief      Release SPI resource
 *@par        External public functions
 *
 *@param      serial_dev_path		path of spi device file
 *
 *@retval     void
 *
**********************************************************************/
void spi_close( void );

/*!********************************************************************
 *@brief      Write data to register through SPI
 *@par        External public functions
 *
 *@param      reg_addr	register address
 *@param      data		data to send
 *
 *@retval     D_RESULT_SUCCESS	write successfully
 *@retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int spi_write_reg_32( unsigned int reg_addr, unsigned int data );

/*!********************************************************************
 *@brief      Read data from register through SPI
 *@par        External public functions
 *
 *@param      reg_addr	register address
 *@param      data		pointer to the area to store data
 *
 *@retval     D_RESULT_SUCCESS	read successfully
 *@retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int spi_read_reg_32( unsigned int reg_addr, unsigned int *data );

/*!********************************************************************
 *@brief      Write data to register continuously through spi
 *@par        External public functions
 *
 *@param      reg_addr		register address
 *@param      write_buff		pointer to the area of source data
 *@param      write_size		data size to write
 *
 *@retval     D_RESULT_SUCCESS	write successfully
 *@retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int spi_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/*!********************************************************************
 *@brief      Read data from register continuously through spi
 *@par        External public functions
 *
 *@param      reg_addr		register address
 *@param      read_buff		pointer to the area to store data
 *@param      write_size		data size to read
 *
 *@retval     D_RESULT_SUCCESS	read successfully
 *@retval     D_RESULT_ERROR		error
 *
**********************************************************************/
int spi_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );
