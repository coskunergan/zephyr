/**
*******************************************************************************
* @file         a31t21x_uart.h
* @author       ABOV R&D Division
* @brief        A31T21x UART Driver Header
*
* Copyright 2017 ABOV Semiconductor Co.,Ltd. All rights reserved.
*
* This file is licensed under terms that are found in the LICENSE file
* located at Document directory.
* If this file is delivered or shared without applicable license terms,
* the terms of the BSD-3-Clause license shall be applied.
* Reference: https://opensource.org/licenses/BSD-3-Clause
******************************************************************************/


#ifndef _A31T21X_UART_H_
#define _A31T21X_UART_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "a31t21x.h"

/**
 * UART instance id
 */
enum uart_id
{
    UART_ID_0   = 0,                        /**< UART 0 */
    UART_ID_1   = 1,                        /**< UART 1 */
    UART_ID_MAX = 2,                        /**< UART max */
};

/**
 * UART data bit
 */
enum uart_data_bit
{
    UART_DATA_BIT_5 = 0,                    /**< 5bit mode */
    UART_DATA_BIT_6 = 1,                    /**< 6bit mode */
    UART_DATA_BIT_7 = 2,                    /**< 7bit mode */
    UART_DATA_BIT_8 = 3,                    /**< 8bit mode */
};

/**
 * UART parity bit
 */
enum uart_parity_bit
{
    UART_PARITY_BIT_NONE    = 0,            /**< parity not used */
    UART_PARITY_BIT_ODD     = 1,            /**< odd parity used */
    UART_PARITY_BIT_EVEN    = 2,            /**< even parity used */
    UART_PARITY_BIT_STUCK   = 3,            /**< parity stuck used */
};

/**
 * UART stop bit
 */
enum uart_stop_bit
{
    UART_STOP_BIT_1 = 0,                    /**< 1bit stop */
    UART_STOP_BIT_2 = 1,                    /**< 2bit stop */
};

/**
 * UART line status
 */
enum uart_line_status
{
    UART_LINE_STATUS_TX_EMPTY       = (1 << 6),   /**< transmit data not set */
    UART_LINE_STATUS_TX_READY       = (1 << 5),   /**< ready for transmit */
    UART_LINE_STATUS_BREAK          = (1 << 4),   /**< a break condition detected */
    UART_LINE_STATUS_FRAME_ERROR    = (1 << 3),   /**< a frame error */
    UART_LINE_STATUS_PARITY_ERROR   = (1 << 2),   /**< a parity error */
    UART_LINE_STATUS_OVERRUN_ERROR  = (1 << 1),   /**< a overrun error */
    UART_LINE_STATUS_RX_DONE        = (1 << 0),   /**< data received */
};

/**
 * UART IO type
 */
enum uart_io_type
{
    UART_IO_TYPE_BLOCK      = 0,            /**< blocking */
    UART_IO_TYPE_NON_BLOCK  = 1,            /**< non-blocking */
};

/**
 * UART interrupt type
 */
enum uart_irq_type
{
    UART_IRQ_RX_AVAIL       = 0,            /**< reception available */
    UART_IRQ_TX_READY       = 1,            /**< transmission buffer empty */
    UART_IRQ_LINE_STATUS    = 2,            /**< line status error */
    UART_IRQ_TX_EXIT        = 3,            /**< transmission exit */
    UART_IRQ_RX_DMA_DONE    = 4,            /**< reception DMA complete */
    UART_IRQ_TX_DMA_DONE    = 5,            /**< transmission DMA complete */
};

/**
 *******************************************************************************
 * @brief       UART interrupt callback function type
 * @param[in]   event event type ::uart_event
 * @param[in]   context context provided during initialization
 ******************************************************************************/
typedef void (*uart_irq_handler_t)(uint32_t event, void *context);

/**
 * UART configuration
 */
struct uart_cfg
{
    uint32_t                baudrate;   /**< baud rate */
    enum uart_data_bit      data_bit;   /**< data bit */
    enum uart_parity_bit    parity_bit; /**< parity bit */
    enum uart_stop_bit      stop_bit;   /**< stop bit */
    uint8_t                 irq_prio;   /**< interrupt priority */
};


#ifdef __cplusplus
}
#endif

#endif /* _A31T21X_UART_H_ */

/** @} */
/** @} */

