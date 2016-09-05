/*!******************************************************************************
 * @file    common.c
 * @brief   common function
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include "common.h"

/*!********************************************************************
 *@brief      Function for endian conversion(4byte)
 *@par        External public functions
 *
 *@param      src    pointer to the source data
 *
 *@retval     void
 *
**********************************************************************/
void common_changeEndian( unsigned int* src )
{
    union {
        unsigned int ui;
        char c[4];
    } src_buf, dst_buf;

    if( src == NULL ) {
        return;
    }
    src_buf.ui = *src;
    dst_buf.c[0] = src_buf.c[3];
    dst_buf.c[1] = src_buf.c[2];
    dst_buf.c[2] = src_buf.c[1];
    dst_buf.c[3] = src_buf.c[0];
    *src = dst_buf.ui;
}

/*!********************************************************************
 *@brief      Function for printing pipe information of thread
 *@par        External public functions
 *
 *@param      thif    pointer to the thread pipe I/F
 *
 *@retval     void
 *
**********************************************************************/
void common_print_pipe( thread_if_t *thif )
{
    if( thif == NULL ) {
        DBG_ERR( "thif is NULL\n" );
        return;
    }
    DBG_PRINT( "thif: in[0]=%d, in[1]=%d, out[0]=%d, out[1]=%d\n",
               thif->pipe_in[0], thif->pipe_in[1], thif->pipe_out[0], thif->pipe_out[1] );
}

/*!********************************************************************
 *@brief      Function for printing information of thread event
 *@par        External public functions
 *
 *@param      ev    pointer to the thread event
 *
 *@retval     void
 *
**********************************************************************/
void common_print_event( thread_event_t *ev )
{
    if( ev == NULL ) {
        DBG_ERR( "ev is NULL\n" );
        return;
    }
    DBG_PRINT( "ev: id=%d, data=%d\n", ev->id, ev->data );
}
