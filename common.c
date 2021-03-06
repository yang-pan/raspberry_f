/*!******************************************************************************
 * @file    common.c
 * @brief   common function
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include "common.h"

// デバッグ用メッセージ出力
#ifdef D_DBG_PRINT_ENABLE
#define DBG_PRINT(...)	printf("%s(%d): ", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// エラー用メッセージ出力
#ifdef D_DBG_ERR_ENABLE
#define DBG_ERR(...)	fprintf(stderr, "[ERR] %s(%d): ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG_ERR(...)
#endif

/**
 * エンディアン変換(4byte)
 */
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

/**
 * スレッド間インタフェース情報画面出力
 */
void common_print_pipe( thread_if_t *thif )
{
	if( thif == NULL ) {
		DBG_ERR( "thif is NULL\n" );
		return;
	}
	DBG_PRINT( "thif: in[0]=%d, in[1]=%d, out[0]=%d, out[1]=%d\n",
	        thif->pipe_in[0], thif->pipe_in[1], thif->pipe_out[0], thif->pipe_out[1] );
}

/**
 * イベント内容画面出力
 */
void common_print_event( thread_event_t *ev )
{
	if( ev == NULL ) {
		DBG_ERR( "ev is NULL\n" );
		return;
	}
	DBG_PRINT( "ev: id=%d, data=%d\n", ev->id, ev->data );
}
