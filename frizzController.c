/*!******************************************************************************
 * @file    frizzController.c
 * @brief   thread for frizz control
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
#include <poll.h>
#include <wiringPi.h>
#include <errno.h>

#include "common.h"
#include "sensor_buff.h"
#include "frizzController.h"
#include "frizzDriver.h"
#include "serial.h"

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

#ifdef D_USE_GPIO_IRQ
#define D_RPI_GPIO_IRQ_PIN	(17) // ラズベリーパイの割込み用GPIOピン番号
static int pipe_gpio_irq[2];	// GPIO割込みからのコマンド受信用パイプ
#endif

static thread_if_t *pIfToMain;	// メインスレッドとの通信インターフェース

/**
 *  GPIO割込みハンドラ
 *  frizzからセンサデータ割込みを受けたタイミングで呼ばれる
 */
#ifdef D_USE_GPIO_IRQ
static void gpio_irq_handler( void )
{
	thread_event_t ev;
	ev.id = EVENT_FRIZZCTRL_GPIO_IRQ;
	ev.data = 0;
	write( pipe_gpio_irq[D_PIPE_W], &ev, sizeof( ev ) );
}
#endif

/**
  * frizz Cotroller 初期化
  */
static int init_frizz_controller( frizzCntrollerArg *arg )
{
	int ver, ret;

	pIfToMain = &arg->thif;
	DBG_PRINT( "SPI device         : %s\n", arg->spi_dev_path );
	DBG_PRINT( "frizz firmware path: %s\n", arg->frizz_firmware_path );

	// センサバッファを初期化
	ret = senbuff_init();
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "sensor buffer initialize failed\n" );
		return D_RESULT_ERROR;
	}

	// SPIを初期化
	ret = serial_open( arg->spi_dev_path );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "serial open failed\n" );
		return D_RESULT_ERROR;
	}

	// frizzのバージョンレジスタを読み出す
	ver = frizzdrv_read_ver_reg();
	DBG_PRINT( "frizz version      : 0x%08x\n", ver );
	if( ver != D_FRIZZ_CHIPID ) {
		DBG_ERR( "reading frizz ver register failed\n" );
		return D_RESULT_ERROR;
	}

	// frizzを初期化する
	ret = frizzdrv_frizz_fw_download( arg->frizz_firmware_path );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "frizz firmware download failed\n" );
		return D_RESULT_ERROR;
	}

#ifdef D_USE_GPIO_IRQ
	// 割り込みハンドラからコマンドを受診するためのパイプを初期化
	if( pipe( pipe_gpio_irq ) != 0 ) {
		DBG_ERR( "pipe error. errno=%d. %s\n", errno, strerror( errno ) );
		return D_RESULT_ERROR;
	}

	// 割込み設定(GPIO 1, Active Low)
	wiringPiSetupSys();
	wiringPiISR( D_RPI_GPIO_IRQ_PIN, INT_EDGE_FALLING, gpio_irq_handler );

	// GPIOによる通知を有効化(GPIO 1, Active Low)
	ret = frizzdrv_set_setting( D_FRIZZ_GPIO_INT_NUM_1, D_FRIZZ_INT_ACTIVE_LOW );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "frizz GPIO setting failed\n" );
		return D_RESULT_ERROR;
	}
#endif

	return D_RESULT_SUCCESS;
}

/**
  * メインスレッドにイベントを送信する
  */
static int sendEvent_toMain( thread_event_t* ev )
{
	if( write( pIfToMain->pipe_out[D_PIPE_W], ev, sizeof( thread_event_t ) ) <= 0 ) {
		DBG_ERR( "write error\n" );
		return D_RESULT_ERROR;
	}
	return D_RESULT_SUCCESS;
}

/**
 * frizzコントローラスレッドメイン関数
 */
void *frizzctrl_main( void *arg )
{
	int ret;
	thread_event_t ev;
	DBG_PRINT( "frizz controller thread: start\n" );


	//-------------------------------------------------------------------------
	// パラメータチェック
	if( arg == NULL ) {
		DBG_PRINT( "arg is NULL. frizz controller thread: end\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// 初期化
	if( init_frizz_controller( ( frizzCntrollerArg* ) arg ) != D_RESULT_SUCCESS ) {
		DBG_ERR( "init_frizz_controller error\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// 初期化完了通知をメインスレッドに送る
	ev.id = EVENT_INITIALIZE_DONE;
	ev.data = 0;
	ret = sendEvent_toMain( &ev );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "sending initialize done event to main failed\n" );
		exit( EXIT_FAILURE );
	}

	//-------------------------------------------------------------------------
	// frizzセンサアクティベート
	// センサアクティベート(加速度)
	ret = frizzdrv_activate( SENSOR_ID_ACCEL_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate acc failed\n" );
		exit( EXIT_FAILURE );
	}

	// センサアクティベート(ジャイロ)
	ret = frizzdrv_activate( SENSOR_ID_GYRO_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate gyro failed\n" );
		exit( EXIT_FAILURE );
	}

	// センサアクティベート(ecompass)
	ret = frizzdrv_activate( SENSOR_ID_MAGNET_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate ecompass failed\n" );
		exit( EXIT_FAILURE );
	}

	// センサアクティベート(Pressure)
	ret = frizzdrv_activate( SENSOR_ID_PRESSURE_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate pressure failed\n" );
		exit( EXIT_FAILURE );
	}

	// センサアクティベート(ECG)
	ret = frizzdrv_activate( SENSOR_ID_ECG_RAW, D_FRIZZ_SENSOR_ACTIVATE, D_FRIZZ_ACTIVATE_PARAM_USE_HWFIFO, D_FRIZZ_ACTIVATE_PARAM_WITH_INTERRUPT );
	if( ret != D_RESULT_SUCCESS ) {
		DBG_ERR( "activate ecg failed\n" );
		exit( EXIT_FAILURE );
	}

#ifdef D_USE_GPIO_IRQ
	struct pollfd fds[2];

	fds[0].fd = pipe_gpio_irq[D_PIPE_R];
	fds[0].events = POLLIN;
	fds[0].revents = 0;

	fds[1].fd = pIfToMain->pipe_in[D_PIPE_R];
	fds[1].events = POLLIN;
	fds[1].revents = 0;

	//メッセージループ
	while( 1 ) {
		poll( fds , ARRAY_SIZE( fds ), -1 );

		// Message from GPIO IRQ
		if( fds[0].revents & POLLIN ) {
			do {
				if( read( fds[0].fd, &ev, sizeof( ev ) ) < 0 ) {
					break;
				}
				fds[0].revents = 0;
				poll( &fds[0] , 1, 0 );
			} while( fds[0].revents & POLLIN );

			// GPIO割込みが発生した場合センサデータを読み出す
			while( frizzdrv_receive_packet() == D_RESULT_SUCCESS );
			fds[0].revents = 0;
		}
		// Message from Main thread
		if( fds[1].revents & POLLIN ) {
			// メインスレッドからコマンドを受信した場合、スレッドを終了する
			break;
		}
	}
#else
	while( 1 )
	{
		frizzdrv_receive_packet();
	}
#endif

	// 終了処理
	if( arg ) {
		free( arg );
	}
	DBG_PRINT( "frizz controller thread: end\n" );

	return 0;
}

