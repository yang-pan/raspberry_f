/*!******************************************************************************
 * @file    sensor_buff.h
 * @brief   source for sensor buffer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __SENSOR_BUFF_H__
#define __SENSOR_BUFF_H__

#include <sys/time.h>

/**
 *	frizzから取得したイベント
 */
typedef struct {
	int	code;				// frizzから受信したコード
	struct timeval	time;	// 受信したシステム時間
	unsigned int frizz_ms;	// frizzの時間
	unsigned int f32_value[64];	// センサデータ
} sensor_data_t;

/**
 *  センサデータ格納用バッファを初期化する
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 */
int senbuff_init(void);

/**
 *  センサデータ格納用バッファを取得する
 *  成功した場合はバッファのインデックス値を返す。
 *  失敗した場合は-1を返す。
 */
int senbuff_alloc( sensor_data_t** sensor_data );

/**
 *  引数のインデックスに対応したセンサデータ格納用バッファを取得する
 *  未使用バッファでも成功するので注意すること。
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 */
int senbuff_getBuffAddr( unsigned int buff_idx, sensor_data_t** sensor_data );

/**
 *  センサデータ格納用バッファを解放する
 *  成功した場合はD_RESULT_SUCCESSを返す。
 *  失敗した場合はD_RESULT_ERRORを返す。
 *
 */
int senbuff_free( unsigned int buff_idx );

#endif // __SENSOR_BUFF_H__
