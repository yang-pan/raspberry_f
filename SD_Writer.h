/*!******************************************************************************
 * @file    SD_Writer.h
 * @brief   source for SD_Writer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __SD_WRITER_H__
#define __SD_WRITER_H__

#include <limits.h>
#include "common.h"

typedef struct {
	thread_if_t	thif;
	char log_file_path[PATH_MAX];
} sdWriterArg;

void *sdWriter_main( void *arg );
int writeDataToTheMedia(int idx);
#endif	// __SD_WRITER_H__
