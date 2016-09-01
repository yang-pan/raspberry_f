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

/*!
 * serial communication of initialize processing
 *
 * @param[in] path of character device
 * @return 0=success, otherwise 0 =fail
 */
int serial_open( const char* serial_dev_path );

/*!
 * serial communication of termination processing
 *
 * @param[in]
 * @return
 */
void serial_close( void );

/**
 * Write data(4bytes) to register 
 */
int serial_write_reg_32( unsigned int, unsigned int );

/**
 * Read data(4bytes) from register
 */
int serial_read_reg_32( unsigned int, unsigned int* );

/**
 *  Write data to register continuously
 *  data size should be an integer multiple of 4 bytes
 */
int serial_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/**
 *  Read data from register continuously
 *  data size should be and integer multiple of 4 bytes 
 */
int serial_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );

#endif // __SERIAL_H__
