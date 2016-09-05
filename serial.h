/*!******************************************************************************
 * @file    serial.h
 * @brief   source for serial
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __SERIAL_H__
#define __SERIAL_H__

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
int serial_open( const char* serial_dev_path );

/*!********************************************************************
 *@brief      Close serial devcie
 *@par        External public functions
 *
 *@param      void
 *
 *@retval     void
 *
**********************************************************************/
void serial_close( void );

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
int serial_write_reg_32( unsigned int reg_addr, unsigned int data );

/*!********************************************************************
 *@brief      Read register
 *@par        External public functions
 *
 *@param      reg_addr	register address
 *@param      data		pointer to the area to store data
 *
 *@retval     D_RESULT_SUCCESS	read successfully
  @retval     D_RESULT_ERROR		read failed
 *
**********************************************************************/
int serial_read_reg_32( unsigned int reg_addr, unsigned int *data );

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
  @retval     D_RESULT_ERROR		write failed
 *
**********************************************************************/
int serial_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

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
  @retval     D_RESULT_ERROR		read failed
 *
**********************************************************************/
int serial_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );

#endif // __SERIAL_H__
