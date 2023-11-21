/*
 * Copyright (c) 2020, Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0 
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_LPC11U6X_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_LPC11U6X_H_

#include <zephyr/drivers/pinctrl.h>

#define LPC11U6X_UART0_LCR_WLS_5BITS             0
#define LPC11U6X_UART0_LCR_WLS_6BITS             1
#define LPC11U6X_UART0_LCR_WLS_7BITS             2
#define LPC11U6X_UART0_LCR_WLS_8BITS             3
#define LPC11U6X_UART0_LCR_STOP_1BIT             (0 << 2)
#define LPC11U6X_UART0_LCR_STOP_2BIT             (1 << 2)
#define LPC11U6X_UART0_LCR_PARTIY_ENABLE         (1 << 3)
#define LPC11U6X_UART0_LCR_PARTIY_ODD            (0 << 4)
#define LPC11U6X_UART0_LCR_PARTIY_EVEN           (1 << 4)

#define LPC11U6X_UART0_LCR_DLAB                  (1 << 7)

#define LPC11U6X_UART0_FCR_FIFO_EN               (1 << 0)

#define LPC11U6X_UART0_LSR_RDR                   (1 << 0)
#define LPC11U6X_UART0_LSR_OE                    (1 << 1)
#define LPC11U6X_UART0_LSR_PE                    (1 << 2)
#define LPC11U6X_UART0_LSR_FE                    (1 << 3)
#define LPC11U6X_UART0_LSR_BI                    (1 << 4)
#define LPC11U6X_UART0_LSR_THRE                  (1 << 5)
#define LPC11U6X_UART0_LSR_TEMT                  (1 << 6)
#define LPC11U6X_UART0_LSR_RXFE                  (1 << 7)

#define LPC11U6X_UART0_IER_RBRINTEN              (1 << 0)
#define LPC11U6X_UART0_IER_THREINTEN             (1 << 1)
//#define LPC11U6X_UART0_IER_RLSINTEN              (1 << 2)
#define LPC11U6X_UART0_IER_MASK                  (0x33F)

#define LPC11U6X_UART0_IIR_STATUS                (0x1 << 0)
#define LPC11U6X_UART0_IIR_INTID(x)              (((x) >> 1) & 0x7)
//#define LPC11U6X_UART0_IIR_INTID_RLS             0x3
#define LPC11U6X_UART0_IIR_INTID_RDA             0x2
#define LPC11U6X_UART0_IIR_INTID_CTI             0x6
//#define LPC11U6X_UART0_IIR_INTID_THRE            0x1

#define LPC11U6X_UART0_FIFO_SIZE                 16

#define LPC11U6X_UARTX_CFG_ENABLE                (0x1 << 0)
#define LPC11U6X_UARTX_CFG_DATALEN_7BIT          (0x0 << 2)
#define LPC11U6X_UARTX_CFG_DATALEN_8BIT          (0x1 << 2)
#define LPC11U6X_UARTX_CFG_DATALEN_9BIT          (0x2 << 2)
#define LPC11U6X_UARTX_CFG_PARITY_NONE           (0x0 << 4)
#define LPC11U6X_UARTX_CFG_PARITY_EVEN           (0x2 << 4)
#define LPC11U6X_UARTX_CFG_PARITY_ODD            (0x3 << 4)
#define LPC11U6X_UARTX_CFG_STOP_1BIT             (0x0 << 6)
#define LPC11U6X_UARTX_CFG_STOP_2BIT             (0x1 << 6)

#define LPC11U6X_UARTX_CFG_MASK                  (0x00FCDAFD)

#define LPC11U6X_UARTX_STAT_RXRDY                (1 << 0)
#define LPC11U6X_UARTX_STAT_TXRDY                (1 << 2)
#define LPC11U6X_UARTX_STAT_TXIDLE               (1 << 3)
#define LPC11U6X_UARTX_STAT_OVERRUNINT           (1 << 8)
#define LPC11U6X_UARTX_STAT_FRAMERRINT           (1 << 13)
#define LPC11U6X_UARTX_STAT_PARITYERRINT         (1 << 14)

#define LPC11U6X_UARTX_BRG_MASK                  (0xFFFF)

#define LPC11U6X_UARTX_INT_EN_SET_RXRDYEN        (1 << 0)
#define LPC11U6X_UARTX_INT_EN_SET_TXRDYEN        (1 << 2)
#define LPC11U6X_UARTX_INT_EN_SET_OVERRUNEN      (1 << 8)
#define LPC11U6X_UARTX_INT_EN_SET_FRAMERREN      (1 << 13)
#define LPC11U6X_UARTX_INT_EN_SET_PARITYERREN    (1 << 14)
#define LPC11U6X_UARTX_INT_EN_SET_MASK           (0x0001F96D)

#define LPC11U6X_UARTX_INT_EN_CLR_RXRDYCLR       (1 << 0)
#define LPC11U6X_UARTX_INT_EN_CLR_TXRDYCLR       (1 << 2)
#define LPC11U6X_UARTX_INT_EN_CLR_OVERRUNCLR     (1 << 8)
#define LPC11U6X_UARTX_INT_EN_CLR_FRAMERRCLR     (1 << 13)
#define LPC11U6X_UARTX_INT_EN_CLR_PARITYERRCLR   (1 << 14)

#define LPC11U6X_UARTX_INT_STAT_RXRDY            (1 << 0)
#define LPC11U6X_UARTX_INT_STAT_TXRDY            (1 << 2)
#define LPC11U6X_UARTX_INT_STAT_OVERRUN          (1 << 8)
#define LPC11U6X_UARTX_INT_STAT_FRAMERR          (1 << 13)
#define LPC11U6X_UARTX_INT_STAT_PARITYERR        (1 << 14)

#define LPC11U6X_UARTX_DEVICE_PER_IRQ            2

// struct lpc11u6x_uart0_regs {
// 	union {
// 		volatile const uint32_t rbr; /* RX buffer (RO) */
// 		volatile uint32_t thr;       /* TX buffer (WO) */
// 		volatile uint32_t dll;       /* Divisor latch LSB */
// 	};
// 	union {
// 		volatile uint32_t dlm;       /* Divisor latch MSB */
// 		volatile uint32_t ier;       /* Interrupt enable */
// 	};
// 	union {
// 		volatile uint32_t iir;       /* Interrupt ID */
// 		volatile uint32_t fcr;       /* FIFO Control */
// 	};
// 	volatile uint32_t lcr;               /* Line Control */
// 	volatile uint32_t mcr;               /* Modem Control */
// 	volatile uint32_t lsr;               /* Line Status */
// 	volatile uint32_t msr;               /* Modem Status */
// 	volatile uint32_t scr;               /* Scratch pad */
// 	volatile uint32_t acr;               /* Auto-baud Control */
// 	volatile uint32_t icr;               /* IrDA Control */
// 	volatile uint32_t fdr;               /* Fractional Divider */
// 	volatile uint32_t osr;               /* Oversampling register */
// 	volatile uint32_t ter;               /* Transmit enable */
// 	volatile uint32_t reserved1[3];
// 	volatile uint32_t hden;              /* Half duplex */
// 	volatile uint32_t reserved2;
// 	volatile uint32_t sci_ctrl;          /* Smart card interface */
// 	volatile uint32_t rs485_ctrl;        /* RS-485 control */
// 	volatile uint32_t rs485_addr_match;  /* RS-485 address match */
// 	volatile uint32_t rs485_dly;         /* RS-485 delay direction control
// 					      * delay
// 					      */
// 	volatile uint32_t sync_ctrl;         /* Synchronous mode control */
// };

struct lpc11u6x_uartx_regs {
  union {
      volatile uint32_t  thr;                             /*!< (@ 0x40004000) Transmit Data Hold Register                            */
      volatile const uint32_t  rbr;                             /*!< (@ 0x40004000) Receive Buffer Register                                */
  };
   volatile uint32_t  ier;                               /*!< (@ 0x40004004) UART Interrupt Enable Register                         */
   volatile uint32_t  iir;                               /*!< (@ 0x40004008) UART Interrupt ID Register                             */
   volatile uint32_t  lcr;                               /*!< (@ 0x4000400C) UART Line Control Register                             */
   volatile uint32_t  dcr;                               /*!< (@ 0x40004010) UART Data Control Register                             */
   volatile uint32_t  lsr;                               /*!< (@ 0x40004014) UART Line Status Register                              */
   volatile const uint32_t  RESERVED[2];
   volatile uint32_t  bdr;                               /*!< (@ 0x40004020) Baud rate Divisor Latch Register                       */
   volatile uint32_t  bfr;                               /*!< (@ 0x40004024) Baud rate Fraction Counter Register                    */
   volatile const uint32_t  RESERVED1[2];
   volatile uint32_t  IDTR;                              /*!< (@ 0x40004030) Inter-frame Delay Time Register                        */
};

struct lpc11u6x_uartx_config {
	struct lpc11u6x_uartx_regs *base;
	const struct device *clock_dev;
	uint32_t baudrate;
	uint32_t clkid;
	const struct pinctrl_dev_config *pincfg;
};

struct lpc11u6x_uartx_data {
	uint32_t baudrate;
	uint8_t parity;
	uint8_t stop_bits;
	uint8_t data_bits;
	uint8_t flow_ctrl;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb;
	void *cb_data;
	uint32_t cached_iir;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#endif /*ZEPHYR_DRIVERS_SERIAL_UART_LPC11U6X_H_*/
