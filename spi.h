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
 * Initialize SPI
 *
 * @param[in] struct of character device
 * @return 0=success, otherwise 0 =fail
 */
int spi_open( const char *spi_dev_path );

/*!
 * Release SPI
 *
 * @param[in]
 * @return
 */
void spi_close( void );

/*!
 * Write data to register by spi
 *
 * @param[in] register address
 * @param[in] writting data
 * @return 0=sucess, otherwise 0=fail
 */

int spi_write_reg_32( unsigned int reg_addr, unsigned int data );

/*!
 * read data from register by spi
 *
 * @param[in] register address
 * @param[out] reading data
 * @return 0=sucess, otherwise 0=fail
 */
int spi_read_reg_32( unsigned int reg_addr, unsigned int *data );

/**
 *  Write data to register continuously by spi
 *  data size should be integer multipe of 4 bytes
 *  success:D_RESULT_SUCCESS
 *  failed :D_RESULT_ERROR
 */
int spi_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/**
 *  Read data from register continuously by spi
 *  data size should be integer multiple of 4 bytes 
 *  success:D_RESULT_SUCCESS
 *  failed:D_RESULT_ERROR
 */
int spi_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );
