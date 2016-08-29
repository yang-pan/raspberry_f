/*!******************************************************************************
 * @file    FuncTest.c
 * @brief   unit test
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/time.h>
#include <wiringPi.h>
#include "common.h"
#include "frizzDriver.h"
#include "serial.h"
#include "sensor_buff.h"
#include "SD_Writer.h"

// Debug message
#ifdef D_DBG_PRINT_ENABLE
#define DBG_PRINT(...)	printf("%s(%d): ", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// Err message
#ifdef D_DBG_ERR_ENABLE
#define DBG_ERR(...)	fprintf(stderr, "[NG] %s(%d): ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG_ERR(...)
#endif

// n:テスト番号、vとeが同じでない場合エラー出力
#define D_CHECK_RESULT_INT_EQ(n,v,e)	{int t = n;	\
										 if(v != e) { DBG_ERR("No:%d, Value=%d, Expected=%d\n",t,v,e); }}
// n:テスト番号、vとeが同じ場合エラー出力
#define D_CHECK_RESULT_INT_NEQ(n,v,e)	{int t = n;	\
										 if(v == e) { DBG_ERR("No:%d, Value=%d, Expected=%d\n",t,v,e); }}

// n:テスト番号、vとeが同じでない場合エラー出力
#define D_CHECK_RESULT_POINTER_EQ(n,v,e) {int t = n;	\
										 if(v != e) { DBG_ERR("No:%d, Value=%p, Expected=%p\n",t,v,e); }}

// n:テスト番号、vとeが同じ場合エラー出力
#define D_CHECK_RESULT_POINTER_NEQ(n,v,e)	{int t = n;	\
											if(v == e) { DBG_ERR("No:%d, Value=%p, Expected=%p\n",t,v,e); }}

//-----------------------------------------------------------------------------
// テストの有効・無効切り替え
//-----------------------------------------------------------------------------
#define D_TEST_SENBUFF_EN
#define D_TEST_SDWRITER_EN
#define D_TEST_SERIAL_EN
#define D_TEST_FRIZZ_DRIVER_EN

#ifdef D_TEST_SENBUFF_EN
static void test_sensor_buff()
{
	const int buff_num = 1024;
	int ret, i, j;
	int n = 1;
	sensor_data_t* psd = NULL;
	sensor_data_t* psd2 = NULL;

	printf( "sensor buff Test Program Start\n" );

	// 未初期化でNGになること
	ret = senbuff_alloc( &psd );
	D_CHECK_RESULT_INT_EQ( n++, ret, -1 );

	ret = senbuff_getBuffAddr( 0, &psd );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_ERROR );

	ret = senbuff_free( 0 );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_ERROR );

	ret = senbuff_free( 0 );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_ERROR );

	// 初期化
	ret = senbuff_init();
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	for( i = 0; i < buff_num; i++ ) {
		// メモリ確保
		ret = senbuff_alloc( &psd );
		D_CHECK_RESULT_INT_EQ( n++, ret, i );
		D_CHECK_RESULT_POINTER_NEQ( n++, psd, NULL );

		// 構造体の内容をチェック
		D_CHECK_RESULT_INT_EQ( n++, psd->code, 0 );
		D_CHECK_RESULT_INT_EQ( n++, ( int )psd->time.tv_sec, 0 );
		D_CHECK_RESULT_INT_EQ( n++, ( int )psd->time.tv_usec, 0 );
		D_CHECK_RESULT_INT_EQ( n++, psd->frizz_ms, 0 );
		for( j = 0; j < ARRAY_SIZE( psd->f32_value ); j++ ) {
			D_CHECK_RESULT_INT_EQ( n++, psd->f32_value[j], 0 );
		}
		// インデックスからメモリを取得(インデックスはiと同じはず)
		ret = senbuff_getBuffAddr( i, &psd2 );
		D_CHECK_RESULT_POINTER_NEQ( n++, psd2, NULL );
		D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

		// 確保したメモリとインデックスから確保したメモリのポインタ比較
		D_CHECK_RESULT_POINTER_EQ( n++, psd, psd2 );
	}

	// 更に確保しようとするとNGになる
	ret = senbuff_alloc( &psd );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_ERROR );

	for( i = 0; i < buff_num; i++ ) {
		ret = senbuff_free( i );
		D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );
	}
	// 更に解放しようとするとNGになる
	ret = senbuff_free( i );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_ERROR );

	return;
}
#endif

#ifdef D_TEST_SDWRITER_EN
static void test_sd_writer()
{
	int ret;
	sensor_data_t* psd = NULL;
	pthread_t th_sdwriter;
	thread_event_t	sdwriter_event;
	sdWriterArg *sdArg;
	char log_file_path[128] = "./test_file.log";
	float* float_pointer;

	//-------------------------------------------------------------------------
	// SD Writer Thread 起動
	//-------------------------------------------------------------------------
	DBG_PRINT( "Create SD Writer thread\n" );
	// ここで確保したメモリはスレッド内で開放する
	sdArg = malloc( sizeof( sdWriterArg ) );
	if( sdArg == NULL ) {
		DBG_ERR( "malloc error\n" );
		exit(EXIT_FAILURE);
	}
	pipe( sdArg->thif.pipe_in );
	pipe( sdArg->thif.pipe_out );

	// エラーチェックしてないので注意
	strncpy(sdArg->log_file_path, log_file_path, strlen(log_file_path));
	
	if( pthread_create( &th_sdwriter, NULL, sdWriter_main, sdArg ) != 0 ) {
		DBG_ERR( "pthread_create() error\n" );
		exit(EXIT_FAILURE);
	}
	//	DBG_PRINT( "Waiting for SD Writer thread initialize done\n" );
	// SD Writer threadの起動完了を待つ
	memset( &sdwriter_event, 0, sizeof( sdwriter_event ) );
	if( read( sdArg->thif.pipe_out[D_PIPE_R], &sdwriter_event, sizeof( sdwriter_event ) ) < 0 ) {
		DBG_ERR( "read error\n" );
		exit(EXIT_FAILURE);
	}
	if( sdwriter_event.id != EVENT_INITIALIZE_DONE )
	{
		DBG_ERR("SD Writer 初期化失敗\n");
		exit(EXIT_FAILURE);
	}
	DBG_PRINT("SD Writer 起動完了\n");

	ret = senbuff_alloc( &psd );
	psd->code = 0x04B180FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x32123330;
	psd->f32_value[0] = 0x04800200;
	psd->f32_value[1] = 0x05780000;
	writeDataToTheMedia(ret);

	ret = senbuff_alloc( &psd );
	psd->code = 0x058080FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x222399;
	float_pointer = (float*)psd->f32_value;
	*(float_pointer + 0) =  0.0012;
	*(float_pointer + 1) = -0.011;
	*(float_pointer + 2) =  1.001;
	writeDataToTheMedia(ret);

	ret = senbuff_alloc( &psd );
	psd->code = 0x058280FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x3212333;
	float_pointer = (float*)psd->f32_value;
	*(float_pointer + 0) = -0.0054;
	*(float_pointer + 1) =  0.002;
	*(float_pointer + 2) = -0.0001;
	writeDataToTheMedia(ret);

	ret = senbuff_alloc( &psd );
	psd->code = 0x058180FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x22321;
	float_pointer = (float*)psd->f32_value;
	*(float_pointer + 0) =  2.38;
	*(float_pointer + 1) =  3.29;
	*(float_pointer + 2) =  1.001;
	writeDataToTheMedia(ret);

	ret = senbuff_alloc( &psd );
	psd->code = 0x038380FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x244649;
	float_pointer = (float*)psd->f32_value;
	*(float_pointer + 0) = 1001.3204;
	writeDataToTheMedia(ret);

	ret = senbuff_alloc( &psd );
	psd->code = 0x000080FF;
	gettimeofday(&psd->time, NULL);
	psd->frizz_ms = 0x8934989;
	writeDataToTheMedia(ret);


	DBG_PRINT( "Waiting for finishing all threads\n" );
	sleep(1);
	write( sdArg->thif.pipe_in[D_PIPE_W], &ret, sizeof( int ) );
	pthread_join( th_sdwriter, NULL ); /*thread_func()スレッドが終了するのを待機する*/

	DBG_PRINT( "--- ECG_log: end ---\n" );
}
#endif

#ifdef D_TEST_SERIAL_EN
void test_serial( void )
{
	int ret;
	int n = 1;
	unsigned int write_data, read_data;

	// SPIをオープンする
	printf( "===== Serial Test Program Start =====\n" );
	ret = serial_open( "/dev/spidev0.0" );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// バージョンレジスタ読み出し
	read_data = 0;
	ret = serial_read_reg_32( 0x01, &read_data );
	printf( "read_data = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );
	D_CHECK_RESULT_INT_EQ( n++, read_data, 0x00000200 );

	//-------------------------------------------------------------------------
	// ram_addr書き込みテスト
	//-------------------------------------------------------------------------
	// ram_addr 読み出し
	read_data = 0;
	ret = serial_read_reg_32( 0x52, &read_data );
	printf( "r: ram_addr = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ram_addr 書き込み
	read_data = 0;
	write_data = 0x00000123;
	printf( "w: ram_addr = 0x%08x\n", write_data );
	ret = serial_write_reg_32( 0x52, write_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ram_addr 読み出し
	ret = serial_read_reg_32( 0x52, &read_data );
	printf( "r: ram_addr = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// verify
	D_CHECK_RESULT_INT_EQ( n++, read_data, write_data );

	//-------------------------------------------------------------------------
	// ram_data書き込みテスト(0x00000000番地に0x12345678を書込みベリファイ)
	//-------------------------------------------------------------------------
	// ram_addr 書き込み
	read_data = 0;
	write_data = 0x00000000;
	printf( "w: ram_addr = 0x%08x\n", write_data );
	ret = serial_write_reg_32( 0x52, write_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ram_addr 読み出し
	ret = serial_read_reg_32( 0x52, &read_data );
	printf( "r: ram_addr = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );
	D_CHECK_RESULT_INT_EQ( n++, read_data, write_data );

	// ram_data 書き込み
	write_data = 0x12345678;
	ret = serial_write_reg_32( 0x53, write_data );
	printf( "w: ram_data = 0x%08x\n", write_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ram_addr 読み出し
	ret = serial_read_reg_32( 0x52, &read_data );
	printf( "r: ram_addr = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// verify(アドレスは書き込み時に1つ進む)
	D_CHECK_RESULT_INT_EQ( n++, read_data, 0x00000001 );

	// ram_addrが進んでしまっているのでデータを書き込んだ0x00000000に戻す
	// ram_addr 書き込み
	write_data = 0x00000000;
	printf( "w: ram_addr = 0x%08x\n", write_data );
	ret = serial_write_reg_32( 0x52, write_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ram_addr 読み出し
	ret = serial_read_reg_32( 0x52, &read_data );
	printf( "r: ram_addr = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );
	D_CHECK_RESULT_INT_EQ( n++, read_data, write_data );

	// ram_data 読み出し
	ret = serial_read_reg_32( 0x53, &read_data );
	printf( "r: ram_data = 0x%08x\n", read_data );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// verify
	D_CHECK_RESULT_INT_EQ( n++, read_data, 0x12345678 );

	// クローズ
	serial_close();

	printf( "===== Serial Test Program Finish =====\n" );
}
#endif

#ifdef D_TEST_FRIZZ_DRIVER_EN
static int int_cnt = 0;
void gpio_interrupt(void){
	int_cnt++;
}

void test_frizz_driver( void )
{
	int n = 1;
	int ret;
	int pre_int_cnt;

	// SPIをオープンする
	printf( "===== frizz driver Test Program Start =====\n" );
	ret = serial_open( "/dev/spidev0.0" );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	ret = serial_open( "/dev/spidev0.0" );
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// バージョンレジスタ読み出し
	ret = frizzdrv_read_ver_reg();
	D_CHECK_RESULT_INT_EQ( n++, ret, 0x00000200 );

	// ファームウェアダウンロード
	ret = frizzdrv_frizz_fw_download("bin/from.bin");
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// 割込み設定
	int setup = 0;
	setup = wiringPiSetupSys();
	DBG_PRINT("setup:%d\n", setup);
	wiringPiISR( 17, INT_EDGE_FALLING, gpio_interrupt );

	// GPIOによる通知を有効化(GPIO 1, Active Low)
	pre_int_cnt = int_cnt;
	ret = frizzdrv_set_setting(D_FRIZZ_GPIO_INT_NUM_1, D_FRIZZ_INT_ACTIVE_LOW);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// センサアクティベート(加速度)
	ret = frizzdrv_activate(SENSOR_ID_ACCEL_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// センサアクティベート(ジャイロ)
	ret = frizzdrv_activate(SENSOR_ID_GYRO_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// センサアクティベート(ecompass)
	ret = frizzdrv_activate(SENSOR_ID_MAGNET_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// センサアクティベート(Pressure)
	ret = frizzdrv_activate(SENSOR_ID_PRESSURE_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );
	
	// センサアクティベート(ECG)
	ret = frizzdrv_activate(SENSOR_ID_ECG_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT);
	D_CHECK_RESULT_INT_EQ( n++, ret, D_RESULT_SUCCESS );

	// ポーリングでセンサデータを受信(100回割込みを受けたら終了する)
	while(int_cnt < 1000)
	{
		if(int_cnt != pre_int_cnt ){
			frizzdrv_receive_packet();
			pre_int_cnt = int_cnt;
//			printf("int_cnt:%d\n", int_cnt);
		}
		usleep(1);
	}
	printf( "===== frizz driver Test Program Finish =====\n" );
}

#endif

//-----------------------------------------------------------------------------
// テストメイン関数
//-----------------------------------------------------------------------------
int main( int arvc, char* argv[] )
{

#ifdef D_TEST_SENBUFF_EN
	test_sensor_buff();
#endif

#ifdef D_TEST_SDWRITER_EN
	test_sd_writer();
#endif

#ifdef D_TEST_SERIAL_EN
	test_serial();
#endif

#ifdef D_TEST_FRIZZ_DRIVER_EN
	test_frizz_driver();
#endif

	return 0;
}
