/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NUMAKER_M45X_RESET_H
#define ZEPHYR_INCLUDE_DT_BINDINGS_RESET_NUMAKER_M45X_RESET_H

/* Beginning of M451 BSP sys_reg.h reset module copy */

#define NUMAKER_SYS_IPRST0_CHIPRST_Pos           (0)
#define NUMAKER_SYS_IPRST0_CPURST_Pos            (1)
#define NUMAKER_SYS_IPRST0_PDMARST_Pos           (2)
#define NUMAKER_SYS_IPRST0_EBIRST_Pos            (3)
#define NUMAKER_SYS_IPRST0_USBHRST_Pos           (4)
#define NUMAKER_SYS_IPRST0_CRCRST_Pos            (7)

#define NUMAKER_SYS_IPRST1_GPIORST_Pos           (1)
#define NUMAKER_SYS_IPRST1_TMR0RST_Pos           (2)
#define NUMAKER_SYS_IPRST1_TMR1RST_Pos           (3)
#define NUMAKER_SYS_IPRST1_TMR2RST_Pos           (4)
#define NUMAKER_SYS_IPRST1_TMR3RST_Pos           (5)
#define NUMAKER_SYS_IPRST1_ACMP01RST_Pos         (7)
#define NUMAKER_SYS_IPRST1_I2C0RST_Pos           (8)
#define NUMAKER_SYS_IPRST1_I2C1RST_Pos           (9)
#define NUMAKER_SYS_IPRST1_SPI0RST_Pos           (12)
#define NUMAKER_SYS_IPRST1_SPI1RST_Pos           (13)
#define NUMAKER_SYS_IPRST1_SPI2RST_Pos           (14)
#define NUMAKER_SYS_IPRST1_UART0RST_Pos          (16)
#define NUMAKER_SYS_IPRST1_UART1RST_Pos          (17)
#define NUMAKER_SYS_IPRST1_UART2RST_Pos          (18)
#define NUMAKER_SYS_IPRST1_UART3RST_Pos          (19)
#define NUMAKER_SYS_IPRST1_CAN0RST_Pos           (24)
#define NUMAKER_SYS_IPRST1_OTGRST_Pos            (26)
#define NUMAKER_SYS_IPRST1_USBDRST_Pos           (27)
#define NUMAKER_SYS_IPRST1_EADCRST_Pos           (28)

#define NUMAKER_SYS_IPRST2_SC0RST_Pos            (0)
#define NUMAKER_SYS_IPRST2_DACRST_Pos            (12)
#define NUMAKER_SYS_IPRST2_PWM0RST_Pos           (16)
#define NUMAKER_SYS_IPRST2_PWM1RST_Pos           (17)
#define NUMAKER_SYS_IPRST2_TKRST_Pos             (25)

/* End of M451 BSP sys_reg.h reset module copy */

/* Beginning of M451 BSP sys.h reset module copy */

/*---------------------------------------------------------------------------------------------------------*/
/*  Module Reset Control Resister constant definitions.                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define NUMAKER_PDMA_RST            ((0UL<<24) | NUMAKER_SYS_IPRST0_PDMARST_Pos)
#define NUMAKER_EBI_RST             ((0UL<<24) | NUMAKER_SYS_IPRST0_EBIRST_Pos)
#define NUMAKER_USBH_RST            ((0UL<<24) | NUMAKER_SYS_IPRST0_USBHRST_Pos)
#define NUMAKER_CRC_RST             ((0UL<<24) | NUMAKER_SYS_IPRST0_CRCRST_Pos)

#define NUMAKER_GPIO_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_GPIORST_Pos)
#define NUMAKER_TMR0_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_TMR0RST_Pos)
#define NUMAKER_TMR1_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_TMR1RST_Pos)
#define NUMAKER_TMR2_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_TMR2RST_Pos)
#define NUMAKER_TMR3_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_TMR3RST_Pos)
#define NUMAKER_ACMP01_RST          ((4UL<<24) | NUMAKER_SYS_IPRST1_ACMP01RST_Pos)
#define NUMAKER_I2C0_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_I2C0RST_Pos)
#define NUMAKER_I2C1_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_I2C1RST_Pos)
#define NUMAKER_SPI0_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_SPI0RST_Pos)
#define NUMAKER_SPI1_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_SPI1RST_Pos)
#define NUMAKER_SPI2_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_SPI2RST_Pos)
#define NUMAKER_UART0_RST           ((4UL<<24) | NUMAKER_SYS_IPRST1_UART0RST_Pos)
#define NUMAKER_UART1_RST           ((4UL<<24) | NUMAKER_SYS_IPRST1_UART1RST_Pos)
#define NUMAKER_UART2_RST           ((4UL<<24) | NUMAKER_SYS_IPRST1_UART2RST_Pos)
#define NUMAKER_UART3_RST           ((4UL<<24) | NUMAKER_SYS_IPRST1_UART3RST_Pos)
#define NUMAKER_CAN0_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_CAN0RST_Pos)
#define NUMAKER_OTG_RST             ((4UL<<24) | NUMAKER_SYS_IPRST1_OTGRST_Pos)
#define NUMAKER_USBD_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_USBDRST_Pos)
#define NUMAKER_EADC_RST            ((4UL<<24) | NUMAKER_SYS_IPRST1_EADCRST_Pos)

#define NUMAKER_SC0_RST             ((8UL<<24) | NUMAKER_SYS_IPRST2_SC0RST_Pos)
#define NUMAKER_DAC_RST             ((8UL<<24) | NUMAKER_SYS_IPRST2_DACRST_Pos)
#define NUMAKER_PWM0_RST            ((8UL<<24) | NUMAKER_SYS_IPRST2_PWM0RST_Pos)
#define NUMAKER_PWM1_RST            ((8UL<<24) | NUMAKER_SYS_IPRST2_PWM1RST_Pos)
#define NUMAKER_TK_RST              ((8UL<<24) | NUMAKER_SYS_IPRST2_TKRST_Pos)

/* End of M451 BSP sys.h reset module copy */

#endif
