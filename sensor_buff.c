/*!******************************************************************************
 * @file    sensor_buff.c
 * @brief   sensor buffer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "sensor_buff.h"
#include "common.h"

//#undef D_DBG_ERR_ENABLE

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


#define D_BUFF_UNUSE	(0)	// バッファ未使用
#define D_BUFF_USE		(1)	// バッファ使用中
#define D_BUFF_NUM	(1024)	// バッファ数

typedef struct {
	int use; 		// 0: unuse, !0: using
	sensor_data_t sensor_data;
} sensor_data_mem_t;

static sensor_data_mem_t sensor_data_mem[D_BUFF_NUM];
static pthread_mutex_t buff_mutex = PTHREAD_MUTEX_INITIALIZER ;

static int initialized = 0;	// 0:初期化未実施、1:初期化済み

/**
 *  センサデータ格納用バッファを初期化する
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 */
int senbuff_init( void )
{
	int ret;

	// mutex lock
	ret = pthread_mutex_lock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't lock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}

	memset( sensor_data_mem, 0, sizeof( sensor_data_mem ) );
	initialized = 1;

	// mutex unlock
	ret = pthread_mutex_unlock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't unlock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}

	return D_RESULT_SUCCESS;
}

/**
 *  センサデータ格納用バッファを取得する
 *  成功した場合はバッファのインデックス値を返す。
 *  失敗した場合は-1を返す。
 */
int senbuff_alloc( sensor_data_t** sensor_data )
{
	int ret, idx = -1;
	int i;

	// mutex lock
	ret = pthread_mutex_lock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't lock mutex. ret = %d", ret );
		return -1;
	}

	if( initialized == 0 ) {
		DBG_ERR( "Initialize hasn't done\n" );
		goto end;
	}

	// 空きメモリを確保する
	for( i = 0; i < ARRAY_SIZE( sensor_data_mem ); i++ ) {
		if( sensor_data_mem[i].use == D_BUFF_UNUSE ) {
			sensor_data_mem[i].use = D_BUFF_USE;
			*sensor_data = &sensor_data_mem[i].sensor_data;
			idx = i;
			break;
		}
	}
end:
	// mutex unlock
	ret = pthread_mutex_unlock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't unlock mutex. ret = %d", ret );
		return -1;
	}
	return idx;
}

/**
 *  引数のインデックスに対応したセンサデータ格納用バッファを取得する
 *  未使用バッファでも成功するので注意すること。
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 */
int senbuff_getBuffAddr( unsigned int buff_idx, sensor_data_t** sensor_data )
{
	int ret, result = D_RESULT_ERROR;

	if( buff_idx > ARRAY_SIZE( sensor_data_mem ) - 1 ) {
		DBG_ERR( "idx error: idx=%d\n", buff_idx );
		return D_RESULT_ERROR;
	}
	// mutex lock
	ret = pthread_mutex_lock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't lock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}
	result = D_RESULT_ERROR;
	if( initialized == 0 ) {
		DBG_ERR( "Initialize hasn't done\n" );
		goto end;
	}
	// インデックスに対応したバッファのポインタを取得する
	*sensor_data = &sensor_data_mem[buff_idx].sensor_data;
	result = D_RESULT_SUCCESS;
	
end:
	// mutex unlock
	ret = pthread_mutex_unlock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't unlock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}
	return result;
}


/**
 *  センサデータ格納用バッファを解放する
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 *
 */
int senbuff_free( unsigned int buff_idx )
{
	int ret, result;
	if( buff_idx > ARRAY_SIZE( sensor_data_mem ) - 1 ) {
		DBG_ERR( "idx error: idx=%d\n", buff_idx );
		return D_RESULT_ERROR;
	}
	// mutex lock
	ret = pthread_mutex_lock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't lock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}
	result = D_RESULT_ERROR;
	if( initialized == 0 ) {
		DBG_ERR( "Initialize hasn't done\n" );
		goto end;
	}
	// メモリを解放
	sensor_data_mem[buff_idx].use = D_BUFF_UNUSE;
	memset( &sensor_data_mem[buff_idx].sensor_data, 0, sizeof( sensor_data_t ) );
	result = D_RESULT_SUCCESS;
end:
	// mutex unlock
	ret = pthread_mutex_unlock( &buff_mutex );
	if ( ret ) {
		DBG_ERR( "Can't unlock mutex. ret = %d", ret );
		return D_RESULT_ERROR;
	}
	return result;

}
