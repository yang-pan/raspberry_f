/*!******************************************************************************
 * @file    common.h
 * @brief   source for common
 * @par     (C) 2016 MegaChips Corporation - All rights reserved.
 *
 * This software is authored by MegaChips Corporation intellectual property,
 * including the copyrights in all countries in the world.
 * This software is provided under a license to use only with all other rights,
 * including ownership rights, being retained by MegaChips Corporation.
 *******************************************************************************/
#ifndef __COMMON_H__
#define __COMMON_H__

// Macro for printf
#define D_DBG_PRINT_ENABLE	// standard log
#define D_DBG_ERR_ENABLE	// err log

// Debug message
#ifdef D_DBG_PRINT_ENABLE
#define DBG_PRINT(...)	printf("%s(%d): ", __func__, __LINE__); printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#endif

// Err message
#ifdef D_DBG_ERR_ENABLE
#define DBG_ERR(...)	fprintf(stderr, "[ERR] %s(%d): ", __func__, __LINE__); fprintf(stderr, __VA_ARGS__)
#else
#define DBG_ERR(...)
#endif

// Macro for GPIO IRQ (frizz => raspberry)
//#define D_USE_GPIO_IRQ

// array size
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))

// index of pipe
#define D_PIPE_R			(0)	// for reading
#define D_PIPE_W			(1)	// for writting

// return value
#define D_RESULT_SUCCESS	(0)		// success
#define D_RESULT_ERROR		(-1)	// err

/**
 * Thread ID
 */
typedef enum {
    THREAD_MAIN = 0,			// Main Thread
    THREAD_FRIZZ_CONTROLLER,	// frizz controller
    THREAD_NUM,
} ThreadIdx_e;

/**
 * Event ID
 */
enum EventIdx {
    // common
    EVENT_INITIALIZE_DONE = 0,			// initialize thread finished
    EVENT_FINISH_THREAD,				// terminate thread finished
    // frizz Controller
    EVENT_FRIZZCTRL_GPIO_IRQ = 1000,	// GPIO IRQ happend
};

/**
 * Thread I/F Event
 */
typedef struct {
    int id;		// Event ID
    int data;	// Event Data
} thread_event_t;

/**
 * Thread I/F
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
 * print thread I/F info
 */
void common_print_pipe( thread_if_t *thif );

/**
 * print event info
 */
void common_print_event( thread_event_t *ev );

#endif // __COMMON_H__
