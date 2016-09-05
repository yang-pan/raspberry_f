/*!******************************************************************************
 * @file    frizzController.h
 * @brief   source for frizzController
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
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

/**@struct  frizzCntrollerArg
 * @brief   Arguments of frizz thread
 */
typedef struct {
    thread_if_t	thif;					// I/F to Main thread
    char spi_dev_path[PATH_MAX];		// SPI device path
    char frizz_firmware_path[PATH_MAX];	// frizz firmware path
} frizzCntrollerArg;

/*!********************************************************************
 *@brief      Main function of frizz control thread
 *@par        External public functions
 *
 *@param      arg     pointer to the arguments of frizz thread
 *
 *@retval     void
 *
**********************************************************************/
void *frizzctrl_main( void *arg );

#endif	// __FRIZZ_CONTROLLER_H__
