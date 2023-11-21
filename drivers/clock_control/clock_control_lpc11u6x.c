/*
 * Copyright (c) 2020, Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_lpc11u6x_syscon

#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "a31t21x.h"

#include <zephyr/drivers/clock_control/lpc11u6x_clock_control.h>

#include "clock_control_lpc11u6x.h"

struct PCU_Type{                                /*!< (@ 0x40001000) PA Structure                                           */
   volatile uint32_t  MOD;                               /*!< (@ 0x40001000) Port n Mode Register                                   */
   volatile uint32_t  TYP;                               /*!< (@ 0x40001004) Port n Output Type Selection Register                  */
   volatile uint32_t  AFSR1;                             /*!< (@ 0x40001008) Port n Alternative Function Selection Registe                                                   1                                                                     */
   volatile uint32_t  AFSR2;                             /*!< (@ 0x4000100C) Port n Alternative Function Selection Registe                                                   2                                                                     */
   volatile uint32_t  PUPD;                              /*!< (@ 0x40001010) Port n Pull-up/down Resistor Selection Register        */
   volatile uint32_t  INDR;                              /*!< (@ 0x40001014) Port n Input Data Register                             */
   volatile uint32_t  OUTDR;                             /*!< (@ 0x40001018) Port n Output Data Register                            */
   volatile uint32_t  BSR;                               /*!< (@ 0x4000101C) Port n Output Bit Set Register                         */
   volatile uint32_t  BCR;                               /*!< (@ 0x40001020) Port n Output Bit Clear Register                       */
   volatile uint32_t  OUTDMSK;                           /*!< (@ 0x40001024) Port n Output Data Mask Register                       */
   volatile uint32_t  DBCR;                              /*!< (@ 0x40001028) Port n Debounce Control Register                       */
   volatile uint32_t  IER;                               /*!< (@ 0x4000102C) Port n interrupt enable register                       */
   volatile uint32_t  ISR;                               /*!< (@ 0x40001030) Port n interrupt status register                       */
   volatile uint32_t  ICR;                               /*!< (@ 0x40001034) Port n interrupt control register                      */
} ;

#define GPIO_REG_OFFSET 			    	(0x100UL)

enum gpio_port
{
    PORTA       = 0,
    PORTB       = 1,
    PORTC       = 2,
    PORTD       = 3,
    PORTE       = 4,
    PORTF       = 5,
    PORT_MAX    = 6,
};


static inline struct PCU_Type *GPIO_REG(enum gpio_port port)
{
    return ( struct PCU_Type *)(PA_BASE + (GPIO_REG_OFFSET * port));
}

static void syscon_power_up(struct lpc11u6x_syscon_regs *syscon,
			    uint32_t bit, bool enable)
{
	if (enable) {
		syscon->pd_run_cfg = (syscon->pd_run_cfg & ~bit)
			| LPC11U6X_PDRUNCFG_MASK;
	} else {
		syscon->pd_run_cfg = syscon->pd_run_cfg | bit
			| LPC11U6X_PDRUNCFG_MASK;
	}
}

static void syscon_set_pll_src(struct lpc11u6x_syscon_regs *syscon,
			       uint32_t src)
{
	syscon->sys_pll_clk_sel = src;
	syscon->sys_pll_clk_uen = 0;
	syscon->sys_pll_clk_uen = 1;
}

static void set_flash_access_time(uint32_t nr_cycles)
{
	uint32_t *reg = (uint32_t *) LPC11U6X_FLASH_TIMING_REG;

	*reg = (*reg & (~LPC11U6X_FLASH_TIMING_MASK)) | nr_cycles;
}

static void syscon_setup_pll(struct lpc11u6x_syscon_regs *syscon,
			     uint32_t msel, uint32_t psel)
{
	uint32_t val = msel & LPC11U6X_SYS_PLL_CTRL_MSEL_MASK;

	val |= (psel & LPC11U6X_SYS_PLL_CTRL_PSEL_MASK) <<
		LPC11U6X_SYS_PLL_CTRL_PSEL_SHIFT;
	syscon->sys_pll_ctrl = val;
}

static bool syscon_pll_locked(struct lpc11u6x_syscon_regs *syscon)
{
	return (syscon->sys_pll_stat & 0x1) != 0;
}

static void syscon_set_main_clock_source(struct lpc11u6x_syscon_regs *syscon,
					 uint32_t src)
{
	syscon->main_clk_sel = src;
	syscon->main_clk_uen = 0;
	syscon->main_clk_uen = 1;
}

static void syscon_ahb_clock_enable(struct lpc11u6x_syscon_regs *syscon,
				    uint32_t mask, bool enable)
{
	if (enable) {
		syscon->sys_ahb_clk_ctrl |= mask;
	} else {
		syscon->sys_ahb_clk_ctrl &= ~mask;
	}
}

static void syscon_peripheral_reset(struct lpc11u6x_syscon_regs *syscon,
				    uint32_t mask, bool reset)
{
	if (reset) {
		syscon->p_reset_ctrl &= ~mask;
	} else {
		syscon->p_reset_ctrl |= mask;
	}
}
static void syscon_frg_init(struct lpc11u6x_syscon_regs *syscon)
{
	uint32_t div;

	return;

	div = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / LPC11U6X_USART_CLOCK_RATE;
	if (!div) {
		div = 1;
	}
	syscon->frg_clk_div = div;

	syscon_peripheral_reset(syscon, LPC11U6X_PRESET_CTRL_FRG, false);
	syscon->uart_frg_div = 0xFF;
	syscon->uart_frg_mult = ((CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC / div)
				 * 256) / LPC11U6X_USART_CLOCK_RATE;
}

static void syscon_frg_deinit(struct lpc11u6x_syscon_regs *syscon)
{
	syscon->uart_frg_div = 0x0;
	syscon_peripheral_reset(syscon, LPC11U6X_PRESET_CTRL_FRG, true);
}

static int lpc11u6x_clock_control_on(const struct device *dev,
				     clock_control_subsys_t sub_system)
{
	const struct lpc11u6x_syscon_config *cfg = dev->config;
	struct lpc11u6x_syscon_data *data = dev->data;
	uint32_t clk_mask = 0, reset_mask = 0;
	int ret = 0, init_frg = 0;

	return 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	switch ((int) sub_system) {
	case LPC11U6X_CLOCK_I2C0:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_I2C0;
		reset_mask = LPC11U6X_PRESET_CTRL_I2C0;
		break;
	case LPC11U6X_CLOCK_I2C1:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_I2C1;
		reset_mask = LPC11U6X_PRESET_CTRL_I2C1;
		break;
	case LPC11U6X_CLOCK_GPIO:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_GPIO |
			LPC11U6X_SYS_AHB_CLK_CTRL_PINT;
		break;
	case LPC11U6X_CLOCK_USART0:
		cfg->syscon->usart0_clk_div = 1;
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART0;
		break;
	case LPC11U6X_CLOCK_USART1:
		if (!data->frg_in_use++) {
			init_frg = 1;
		}
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART1;
		reset_mask = LPC11U6X_PRESET_CTRL_USART1;
		break;
	case LPC11U6X_CLOCK_USART2:
		if (!data->frg_in_use++) {
			init_frg = 1;
		}
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART2;
		reset_mask = LPC11U6X_PRESET_CTRL_USART2;
		break;
	case LPC11U6X_CLOCK_USART3:
		if (!data->frg_in_use++) {
			init_frg = 1;
		}
		data->usart34_in_use++;
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART3_4;
		reset_mask = LPC11U6X_PRESET_CTRL_USART3;
		break;
	case LPC11U6X_CLOCK_USART4:
		if (!data->frg_in_use++) {
			init_frg = 1;
		}
		data->usart34_in_use++;
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART3_4;
		reset_mask = LPC11U6X_PRESET_CTRL_USART4;
		break;
	default:
		k_mutex_unlock(&data->mutex);
		return -EINVAL;
	}

	syscon_ahb_clock_enable(cfg->syscon, clk_mask, true);
	if (init_frg) {
		syscon_frg_init(cfg->syscon);
	}
	syscon_peripheral_reset(cfg->syscon, reset_mask, false);
	k_mutex_unlock(&data->mutex);

	return ret;
}

static int lpc11u6x_clock_control_off(const struct device *dev,
				      clock_control_subsys_t sub_system)
{
	const struct lpc11u6x_syscon_config *cfg = dev->config;
	struct lpc11u6x_syscon_data *data = dev->data;
	uint32_t clk_mask = 0, reset_mask = 0;
	int ret = 0, deinit_frg = 0;

	return 0;

	k_mutex_lock(&data->mutex, K_FOREVER);

	switch ((int) sub_system) {
	case LPC11U6X_CLOCK_I2C0:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_I2C0;
		reset_mask = LPC11U6X_PRESET_CTRL_I2C0;
		break;
	case LPC11U6X_CLOCK_I2C1:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_I2C1;
		reset_mask = LPC11U6X_PRESET_CTRL_I2C1;
		break;
	case LPC11U6X_CLOCK_GPIO:
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_GPIO |
			LPC11U6X_SYS_AHB_CLK_CTRL_PINT;
		break;
	case LPC11U6X_CLOCK_USART0:
		cfg->syscon->usart0_clk_div = 0;
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART0;
		break;
	case LPC11U6X_CLOCK_USART1:
		if (!(--data->frg_in_use)) {
			deinit_frg = 1;
		}
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART1;
		reset_mask = LPC11U6X_PRESET_CTRL_USART1;
		break;
	case LPC11U6X_CLOCK_USART2:
		if (!(--data->frg_in_use)) {
			deinit_frg = 1;
		}
		clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART2;
		reset_mask = LPC11U6X_PRESET_CTRL_USART2;
		break;
	case LPC11U6X_CLOCK_USART3:
		if (!(--data->frg_in_use)) {
			deinit_frg = 1;
		}
		if (!(--data->usart34_in_use)) {
			clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART3_4;
		}
		reset_mask = LPC11U6X_PRESET_CTRL_USART3;
		break;
	case LPC11U6X_CLOCK_USART4:
		if (!(--data->frg_in_use)) {
			deinit_frg = 1;
		}
		if (!(--data->usart34_in_use)) {
			clk_mask = LPC11U6X_SYS_AHB_CLK_CTRL_USART3_4;
		}
		reset_mask = LPC11U6X_PRESET_CTRL_USART4;
		break;
	default:
		k_mutex_unlock(&data->mutex);
		return -EINVAL;
	}

	syscon_ahb_clock_enable(cfg->syscon, clk_mask, false);
	if (deinit_frg) {
		syscon_frg_deinit(cfg->syscon);
	}
	syscon_peripheral_reset(cfg->syscon, reset_mask, true);
	k_mutex_unlock(&data->mutex);
	return ret;

}

static int lpc11u6x_clock_control_get_rate(const struct device *dev,
					   clock_control_subsys_t sub_system,
					   uint32_t *rate)
{
	switch ((int) sub_system) {
	case LPC11U6X_CLOCK_I2C0:
	case LPC11U6X_CLOCK_I2C1:
	case LPC11U6X_CLOCK_GPIO:
	case LPC11U6X_CLOCK_USART0:
	case LPC11U6X_CLOCK_USART1:
	case LPC11U6X_CLOCK_USART2:
	case LPC11U6X_CLOCK_USART3:
	case LPC11U6X_CLOCK_USART4:
		*rate = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

enum scu_clock
{
    SCU_CLOCK_LSI			= 0,					/**< Internal low speed clcok(500Khz) */
    SCU_CLOCK_LSE			= 1,					/**< External low speed clcok */
    SCU_CLOCK_HSI			= 2,					/**< Internal high speed clock(32Mhz) */
    SCU_CLOCK_HSE			= 6,					/**< Extern high speed clock */
    SCU_CLOCK_PLL			= 7,					/**< PLL clock from HSE 8MHz */
};

/**
 * SCU clock divider
 */
enum scu_clock_div
{
    SCU_CLOCK_DIV_NONE			= 0,    /**< Do not divide clock */
    SCU_CLOCK_DIV_1					= 8,    /**< Divide the clock by 1 */
    SCU_CLOCK_DIV_2					= 9,    /**< Divide the clock by 2 */
    SCU_CLOCK_DIV_4					= 10,   /**< Divide the clock by 4 */
    SCU_CLOCK_DIV_8					= 11,   /**< Divide the clock by 8 */
    SCU_CLOCK_DIV_16				= 12,   /**< Divide the clock by 16 */
    SCU_CLOCK_DIV_32				= 13,   /**< Divide the clock by 32 */
};

static void SCU_ClockEnable(enum scu_clock mclk_sel, enum scu_clock_div mclk_div)
{
    uint32_t reg_val;
    uint32_t delay_1ms;
    volatile uint32_t i;

    delay_1ms = 21 * (16000000 / 500000);

    switch(mclk_sel)
    {
        case SCU_CLOCK_LSI:
            reg_val = SCU->CSCR;
            reg_val &= ~(0x0F << 8);
            reg_val |= (mclk_div << 8);
            SCU->CSCR = reg_val | (0xA507UL << 16);

            for(i = 0; i < delay_1ms; i++);

            reg_val = SCU->SCCR;
            reg_val &= ~(0x07 << 0);
            reg_val |= mclk_sel;
            SCU->SCCR = (0x570AUL << 16) | reg_val;
            break;
        case SCU_CLOCK_LSE:
            reg_val = SCU->CSCR;
            reg_val &= ~(0x0F << 12);
            reg_val |= (mclk_div << 12);
            SCU->CSCR = reg_val | (0xA507UL << 16);

            for(i = 0; i < delay_1ms * 500; i++);

            reg_val = SCU->SCCR;
            reg_val &= ~(0x07 << 0);
            reg_val |= mclk_sel;
            SCU->SCCR = (0x570AUL << 16) | reg_val;
            break;
        case SCU_CLOCK_HSI:
            reg_val = SCU->CSCR;
            reg_val &= ~(0x0F << 4);
            reg_val |= (mclk_div << 4);
            SCU->CSCR = reg_val | (0xA507UL << 16);

            for(i = 0; i < delay_1ms; i++);

            reg_val = SCU->SCCR;
            reg_val &= ~(0x07 << 0);
            reg_val |= mclk_sel;
            SCU->SCCR = (0x570AUL << 16) | reg_val;
            break;
        case SCU_CLOCK_HSE:
            reg_val = SCU->CSCR;
            reg_val &= ~(0x0F << 0);
            reg_val |= (mclk_div << 0);
            SCU->CSCR = reg_val | (0xA507UL << 16);

            for(i = 0; i < delay_1ms * 10; i++);

            reg_val = SCU->SCCR;
            reg_val &= ~(0x07 << 0);
            reg_val |= mclk_sel;
            SCU->SCCR = (0x570AUL << 16) | reg_val;
            break;
        case SCU_CLOCK_PLL:
            // if(g_scb.pclk_src == SCU_PLL_CLOCK_SRC_HSI)
            // {
            //     reg_val = SCU->SCCR;
            //     reg_val &= ~(0x07 << 0);
            //     reg_val |= 0x03;
            //     SCU->SCCR = (0x570AUL << 16) | reg_val;
            // }
            // else
            // {
            //     reg_val = SCU->SCCR;
            //     reg_val &= ~(0x07 << 0);
            //     reg_val |= mclk_sel;
            //     SCU->SCCR = (0x570AUL << 16) | reg_val;
            // }
            break;
        default:
            //DRIVER_ASSERT(0);
            break;
    }
}

static int lpc11u6x_syscon_init(const struct device *dev)
{
	const struct lpc11u6x_syscon_config *cfg = dev->config;
	struct lpc11u6x_syscon_data *data = dev->data;
	uint32_t val;

	/* WDT Disable */
    WDT->CR = (0x5A69 << 16)
              | (0x25 << 10)
              | (0x1A << 4);

    /* GPIO Access Enable */
    PORTEN->EN = 0x15;
    PORTEN->EN = 0x51;

    /* Flash Access Time Configure */
    FMC->MR = 0x81;
    FMC->MR = 0x28;
    FMC->CFG = (0x7858 << 16) | (3 << 8);		// Flash Access in 4 cycles (3-wait)
    FMC->MR = 0;	

    struct PCU_Type *pcu;
    uint8_t i;

    /* Peripheral Enable(0:Disable, 1:Enable) */
    SCU->PER1 = SCU->PER1
                | (1 << 13)	// GPIOF
                | (1 << 12)	// GPIOE
                | (1 << 11)	// GPIOD
                | (1 << 10)	// GPIOC
                | (1 << 9)	// GPIOB
                | (1 << 8)	// GPIOA
                ;
    /* Peripheral Clock Enable(0:Disable, 1:Enable) */ 
    SCU->PCER1 = SCU->PCER1
                 | (1 << 13)	// GPIOF
                 | (1 << 12)	// GPIOE
                 | (1 << 11)	// GPIOD
                 | (1 << 10)	// GPIOC
                 | (1 << 9)	// GPIOB
                 | (1 << 8)	// GPIOA
                 ;

    /* Set Port Output to Low except below functions */
    for(i = 0; i < PORT_MAX; i++)
    {
        // Get Register Name
        pcu = GPIO_REG((enum gpio_port)i);

        // PB3(BOOT)
        if(i == PORTB)
        {
            pcu->OUTDR	= 0;
            pcu->MOD		= 0x55555595UL;
            pcu->AFSR1	= (1 << 12);
            pcu->AFSR2	= 0;
        }
        // PE12(nRESET)
        else if(i == PORTE)
        {
            pcu->OUTDR	= 0;
            pcu->MOD		= 0x56555555UL;
            pcu->AFSR1	= 0;
            pcu->AFSR2	= (1 << 16);
        }
        // PF7(XOUT),PF6(XIN),PF5(SXOUT),PF4(SXIN),PF1(SWCLK),PF0(SWDIO)
        else if(i == PORTF)
        {
            pcu->OUTDR	= 0;
            pcu->MOD		= 0x5555AA5AUL;
            pcu->AFSR1	= (4 << 28) | (4 << 24) | (4 << 20) | (4 << 16) | (3 << 4) | (3 << 0);
            pcu->AFSR2	= 0;
        }
        // Others
        else
        {
            pcu->OUTDR	= 0;
            pcu->MOD		= 0x55555555UL;
            pcu->AFSR1	= 0;
            pcu->AFSR2	= 0;
        }
        pcu->TYP = 0;		// Push-pull Output
        pcu->PUPD = 0;	// Pull-up/down Disable
    }

	SCU_ClockEnable(SCU_CLOCK_HSI, SCU_CLOCK_DIV_1); // 32MHz

	k_mutex_init(&data->mutex);
	return 0;
	data->frg_in_use = 0;
	data->usart34_in_use = 0;
	/* Enable SRAM1 and USB ram if needed */
	val = 0;
#ifdef CONFIG_CLOCK_CONTROL_LPC11U6X_ENABLE_SRAM1
	val |= LPC11U6X_SYS_AHB_CLK_CTRL_SRAM1;
#endif /* CONFIG_CLOCK_CONTROL_LPC11U6X_ENABLE_SRAM1 */
#ifdef CONFIG_CLOCK_CONTROL_LPC11U6X_ENABLE_USB_RAM
	val |= LPC11U6X_SYS_AHB_CLK_CTRL_USB_SRAM;
#endif /* CONFIG_CLOCK_CONTROL_LPC11U6X_ENABLE_USB_RAM */

	/* Enable IOCON (I/O Control) clock. */
	val |= LPC11U6X_SYS_AHB_CLK_CTRL_IOCON;

	syscon_ahb_clock_enable(cfg->syscon, val, true);

	/* Configure PLL output as the main clock source, with a frequency of
	 * 48MHz
	 */
#ifdef CONFIG_CLOCK_CONTROL_LPC11U6X_PLL_SRC_SYSOSC
	syscon_power_up(cfg->syscon, LPC11U6X_PDRUNCFG_SYSOSC_PD, true);

	/* Wait ~500us */
	for (int i = 0; i < 2500; i++) {
	}

	/* Configure PLL input */
	syscon_set_pll_src(cfg->syscon, LPC11U6X_SYS_PLL_CLK_SEL_SYSOSC);

	pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);

#elif defined(CONFIG_CLOCK_CONTROL_LPC11U6X_PLL_SRC_IRC)
	syscon_power_up(cfg->syscon, LPC11U6X_PDRUNCFG_IRC_PD, true);
	syscon_set_pll_src(cfg->syscon, LPC11U6X_SYS_PLL_CLK_SEL_IRC);
#endif
	/* Flash access takes 3 clock cycles for main clock frequencies
	 * between 40MHz and 50MHz
	 */
	set_flash_access_time(LPC11U6X_FLASH_TIMING_3CYCLES);

	/* Shutdown PLL to change divider/mult ratios */
	syscon_power_up(cfg->syscon, LPC11U6X_PDRUNCFG_PLL_PD, false);

	/* Setup PLL to have 48MHz output */
	syscon_setup_pll(cfg->syscon, 3, 1);

	/* Power up pll and wait */
	syscon_power_up(cfg->syscon, LPC11U6X_PDRUNCFG_PLL_PD, true);

	while (!syscon_pll_locked(cfg->syscon)) {
	}

	cfg->syscon->sys_ahb_clk_div = 1;
	syscon_set_main_clock_source(cfg->syscon, LPC11U6X_MAIN_CLK_SRC_PLLOUT);
	return 0;
}

static const struct clock_control_driver_api lpc11u6x_clock_control_api = {
	.on = lpc11u6x_clock_control_on,
	.off = lpc11u6x_clock_control_off,
	.get_rate = lpc11u6x_clock_control_get_rate,
};


PINCTRL_DT_INST_DEFINE(0);

static const struct lpc11u6x_syscon_config syscon_config = {
	.syscon = (struct lpc11u6x_syscon_regs *) DT_INST_REG_ADDR(0),
	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

static struct lpc11u6x_syscon_data syscon_data;

DEVICE_DT_INST_DEFINE(0,
		    &lpc11u6x_syscon_init,
		    NULL,
		    &syscon_data, &syscon_config,
		    PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &lpc11u6x_clock_control_api);
