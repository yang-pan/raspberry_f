/*!******************************************************************************
 * @file    spi.h
 * @brief   source for spi
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
/*!
 * SPI initialize processing
 *
 * @param[in] struct of character device
 * @return 0=success, otherwise 0 =fail
 */
int spi_open( const char *spi_dev_path );

/*!
 * SPI termination processing
 *
 * @param[in]
 * @return
 */
void spi_close( void );

/*!
 * SPI 32bit data of writing processing
 *
 * @param[in] register address
 * @param[in] writing data
 * @return 0=sucess, otherwise 0=fail
 */
int spi_write_reg_32( unsigned int reg_addr, unsigned int data );

/*!
 * SPI 32bit data of reading processing
 *
 * @param[in] register address
 * @param[out] reading data
 * @return 0=sucess, otherwise 0=fail
 */
int spi_read_reg_32( unsigned int reg_addr, unsigned int *data );

/**
 *  指定したレジスタに対し連続して書込む
 *  書き込みデータのバイト数は4の倍数でなければならない
 *  正常時：D_RESULT_SUCCESSを返す
 *  失敗時：D_RESULT_ERRORを返す
 */
int spi_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/**
 *  指定したレジスタから指定バイト数分データを読み出す
 *  バイト数は4の倍数でなければならない
 *  正常時：D_RESULT_SUCCESSを返す
 *  失敗時：D_RESULT_ERRORを返す
 */
int spi_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );
