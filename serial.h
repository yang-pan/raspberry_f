/*!******************************************************************************
 * @file    serial.h
 * @brief   source for serial
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
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
 * 指定したレジスタに値を書込む
 */
int serial_write_reg_32( unsigned int, unsigned int );

/**
 * 指定したレジスタの値を読み出す
 */
int serial_read_reg_32( unsigned int, unsigned int* );

/**
 *  指定したレジスタに対し連続して書込む
 *  書き込みデータのバイト数は4の倍数でなければならない
 */
int serial_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/**
 *  指定したレジスタから指定バイト数分データを読み出す
 *  バイト数は4の倍数でなければならない
 */
int serial_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );

#endif // __SERIAL_H__
