/*!******************************************************************************
 * @file    frizzController.h
 * @brief   source for frizzController
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __FRIZZ_CONTROLLER_H__
#define __FRIZZ_CONTROLLER_H__

#include <limits.h>
#include "common.h"

/**
 * frizz thread's argument
 */
typedef struct {
	thread_if_t	thif;				// I/F to Main thread
	char spi_dev_path[PATH_MAX];	// SPI device path
	char frizz_firmware_path[PATH_MAX];	// frizz firmware path
} frizzCntrollerArg;


/**
 * frizz control thread's main function 
 */
void *frizzctrl_main( void *arg );

#endif	// __FRIZZ_CONTROLLER_H__
