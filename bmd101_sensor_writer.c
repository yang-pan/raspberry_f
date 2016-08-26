/*!******************************************************************************
 * @file    bmd101_sensor_writer.c
 * @brief   bmd101 sensor data writer
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include "bmd101_sensor_writer.h"
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
  * BMD101のSensorDataからlengthを取り出す
  */
static unsigned char getBmd101Length( sensor_data_t* recv_data )
{
	unsigned char length;
	length = 0xFF & recv_data->f32_value[0];
	return ( length );
}

void BMD101SensorWriter( sensor_data_t* recv_data, FILE* fp )
{
	unsigned char plength;
	unsigned int timestamp;
	timestamp = recv_data->frizz_ms;
	plength = getBmd101Length( recv_data );
	// Write timestamp
	ucharWriter( ( unsigned char* )&timestamp, sizeof( timestamp ), fp );
	
	// Write Sensor Data(BMD101 Sensor Data includes "Length(1byte)" and "Checksum(1byte)")
	ucharWriter( ( unsigned char* )recv_data->f32_value, plength + 2, fp );

//#define D_BMD_SENSORWRITER_PRINT_DATA
#ifdef D_BMD_SENSORWRITER_PRINT_DATA
	{
		int i;
		unsigned char *p;
		int num;
		num = 0xFF & recv_data->code;
		DBG_PRINT( " header: 0x%08X num:%d timestamp: 0x%08X data: ", recv_data->code, num, recv_data->frizz_ms );
		for( i = 0; i < num - 1; i++ ) {
			p = ( unsigned char* )&recv_data->f32_value[i];
			printf( "%02X %02X %02X %02X ", *( p + 0 ), *( p + 1 ), *( p + 2 ), *( p + 3 ) );
		}
		printf( "\n" );
	}
#endif
}
