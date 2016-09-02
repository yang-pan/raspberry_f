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
/**
 * Initialize SPI
 */
int spi_open( const char *spi_dev_path );

/*!
 * Release SPI
 */
void spi_close( void );

/*!
 * Write data to register by spi
 */
int spi_write_reg_32( unsigned int reg_addr, unsigned int data );

/*!
 * read data from register by spi
 */
int spi_read_reg_32( unsigned int reg_addr, unsigned int *data );

/**
 *  Write data to register continuously by spi
 *  data size should be integer multipe of 4 bytes
 */
int spi_write_burst( unsigned int reg_addr, unsigned char *write_buff, int write_size );

/**
 *  Read data from register continuously by spi
 *  data size should be integer multiple of 4 bytes
 */
int spi_read_burst( unsigned int reg_addr, unsigned char *read_buff, int read_size );
