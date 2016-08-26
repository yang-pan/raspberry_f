/*!******************************************************************************
 * @file    normal_sensor_writer.c
 * @brief   sensor data ( accel, gyro, ecompass, pressure ) writer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include "normal_sensor_writer.h"

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
  * unsigned charの値をファイルに保存する
  * lengthは最大255でなければならない
  */
void ucharWriter( const unsigned char* write_data, unsigned char length, FILE* fp )
{
	int write_size = 0;
	while( length ) {
		write_size += fwrite( write_data + write_size, sizeof( unsigned char ), length, fp );
		length -= write_size;
	}
}

/**
  * センサデータのcodeから、センサデータのバイト数を算出する
  */
unsigned char getSensorDataLength( sensor_data_t* recv_data )
{
	int code;
	unsigned char num;
	code = recv_data->code;
	num = 0xFF & code;	// numはtimetamp(1)とセンサデータ数の合計値
	num -= 1; // timestamp分を引く
	return ( num * 4 );	// 1[word] = 4[byte]
}

/**
  * sendor dataからcheckSumを計算する
  */
unsigned char calcCheckSum( sensor_data_t* recv_data )
{
	unsigned char checksum = 0;
	unsigned char length;
	unsigned char* sensordata;
	int i;
	length = getSensorDataLength( recv_data );
	sensordata = ( unsigned char* )recv_data->f32_value;
	for( i = 0; i < length; i++ ) {
		checksum += *( sensordata + i );
	}
	return ( ~checksum );
}

/**
 * 加工(パディングや不要センサデータの削除など)が必要ないセンサデータを
 * ファイルに書き出す
 */
void normalSensorWriter( sensor_data_t* recv_data, FILE* fp )
{
	unsigned char length;
	unsigned int timestamp;
	unsigned char checksum;

	// 書き込みデータ作成
	// timestamp
	timestamp = recv_data->frizz_ms;
	// length
	length = getSensorDataLength( recv_data );
	// checksum
	checksum = calcCheckSum( recv_data );

	// データ書き込み
	ucharWriter( ( unsigned char* )&timestamp, 4, fp );
	ucharWriter( &length, 1, fp );
	ucharWriter( ( unsigned char* )recv_data->f32_value, length, fp );
	ucharWriter( &checksum, 1, fp );

//#define D_NORMAL_SENSORWRITER_PRINT_DATA
#ifdef D_NORMAL_SENSORWRITER_PRINT_DATA
	{
		int i;
		unsigned char *p;
		int num;
		num = 0xFF & recv_data->code;
		DBG_PRINT( " header: 0x%08X num:%d timestamp: 0x%08X data: ", recv_data->code, num, recv_data->frizz_ms );
		printf("%02X ", length);
		for( i = 0; i < num - 1; i++ ) {
			p = ( unsigned char* )&recv_data->f32_value[i];
			printf( "%02X %02X %02X %02X ", *( p + 0 ), *( p + 1 ), *( p + 2 ), *( p + 3 ) );
		}
		printf("%02X ", checksum);
		printf( "\n" );
	}
#endif
}
