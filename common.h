/*!******************************************************************************
 * @file    common.h
 * @brief   source for common
 * @par     (C) 2015 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __COMMON_H__
#define __COMMON_H__

// プリント文有効化マクロ
#define D_DBG_PRINT_ENABLE	// 標準ログ
#define D_DBG_ERR_ENABLE	// エラーログ

// frizzからのGPIO割込みを使用
//#define D_USE_GPIO_IRQ

// 配列要素数算出マクロ
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

// パイプインデックス
#define D_PIPE_R	(0)	// 読み込み用
#define D_PIPE_W	(1)	// 書き込み用

// 関数返り値
#define D_RESULT_SUCCESS	(0)		// 正常終了
#define D_RESULT_ERROR		(-1)	// エラー

/**
 * Thread ID
 */
typedef enum {
    THREAD_MAIN = 0,			// Main Thread
    THREAD_FRIZZ_CONTROLLER,	// frizz controller
    THREAD_SD_WRITER,			// SD Writer
    THREAD_NUM,
} ThreadIdx_e;

/**
 *  Event ID
 */
enum EventIdx {
	// 共通
    EVENT_INITIALIZE_DONE = 0,		// thread 起動完了
    EVENT_FINISH_THREAD,			// thread 終了
    // frizz Controller用
    EVENT_FRIZZCTRL_GPIO_IRQ = 1000,	// GPIO割込み発生
};

/**
 * Thread I/F Event
 */
typedef struct {
	int id;		// Event ID
	int data;	// Event Data
} thread_event_t;

/**
 *	Thread I/F
 */
typedef struct {
	int pipe_in[2];
	int pipe_out[2];
} thread_if_t;

/**
 * Change endian(4byte)
 */
void common_changeEndian( unsigned int* src );

/**
 * スレッド間インタフェース情報画面出力
 */
void common_print_pipe( thread_if_t *thif );

/**
 * イベント内容画面出力
 */
void common_print_event( thread_event_t *ev );

#endif // __COMMON_H__
