/**************************************************************************//**
 * @file config.h
 * @brief Bootloader Configuration.
 *    This file defines how the bootloader is set up.
 * @author Silicon Labs
 * @version x.xx
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#ifndef CONFIG_H
#define CONFIG_H

/******************************************************************************
 ** DEBUG #define's                                                          **
 ******************************************************************************/
/* This #define pulls in printf etc. Undef when making release build. */
//#define BL_DEBUG

/* This #define simulates that the SWCLK pin is pulled high.
 * Undef when making release build.                                   */
//#define SIMULATE_SWCLK_PIN_HI

/******************************************************************************
 ** Bootloader UART on SWD pins (overlap).                                   **
 ******************************************************************************/
#if defined( _EFM32_HAPPY_FAMILY )
#define USART_OVERLAPS_WITH_BOOTLOADER
#endif

/******************************************************************************
 ** Define USB endpoint addresses                                            **
 ******************************************************************************/
#define EP_DATA_OUT  0x01       /* Endpoint for USB data reception.       */
#define EP_DATA_IN   0x81       /* Endpoint for USB data transmission.    */
#define EP_NOTIFY    0x82       /* The notification endpoint (not used).  */

/******************************************************************************
 ** Number of milliseconds between each consecutive polling of the SWD pins  **
 ******************************************************************************/
#define PIN_LOOP_INTERVAL    250

/******************************************************************************
 * USART used for communication.                                              *
 ******************************************************************************/
#if defined( _EZR32_LEOPARD_FAMILY )
#define BOOTLOADER_USART           USART1
#define BOOTLOADER_USART_CLOCKEN   CMU_HFPERCLKEN0_USART1
#define BOOTLOADER_USART_LOCATION  USART_ROUTE_LOCATION_LOC2

#elif defined( _EFM32_HAPPY_FAMILY )
#define BOOTLOADER_USART           LEUART0
#define BOOTLOADER_USART_CLOCK     cmuClock_LEUART0
#define BOOTLOADER_USART_CLOCKEN   0
#define BOOTLOADER_LEUART_CLOCKEN  CMU_LFBCLKEN0_LEUART0
#define BOOTLOADER_USART_LOCATION  LEUART_ROUTE_LOCATION_LOC3

#else
#define BOOTLOADER_USART           USART0
#define BOOTLOADER_USART_CLOCKEN   CMU_HFPERCLKEN0_USART0
#define BOOTLOADER_USART_LOCATION  USART_ROUTE_LOCATION_LOC0
#endif

/******************************************************************************
 * TIMERn is used for autobaud. The channel and location must match the       *
 * RX line of BOOTLOADER_USART for this to work properly.                     *
 ******************************************************************************/
#if defined( _EFM32_HAPPY_FAMILY )
#define AUTOBAUD_TIMER_IRQHANDLER  TIMER0_IRQHandler
#define AUTOBAUD_TIMER             TIMER0
#define AUTOBAUD_TIMER_IRQn        TIMER0_IRQn
#define AUTOBAUD_TIMER_CLOCK       CMU_HFPERCLKEN0_TIMER0
#define AUTOBAUD_TIMER_CHANNEL     1
#define AUTOBAUD_TIMER_LOCATION    TIMER_ROUTE_LOCATION_LOC5
#define AUTOBAUD_TIMER_INT_MASK    TIMER_IFC_CC1
#define AUTOBAUD_TIMER_ROUTE       TIMER_ROUTE_CC1PEN

#else
#define AUTOBAUD_TIMER_IRQHANDLER  TIMER1_IRQHandler
#define AUTOBAUD_TIMER             TIMER1
#define AUTOBAUD_TIMER_IRQn        TIMER1_IRQn
#define AUTOBAUD_TIMER_CLOCK       CMU_HFPERCLKEN0_TIMER1

#if defined( _EZR32_LEOPARD_FAMILY )
#define AUTOBAUD_TIMER_CHANNEL     0
#define AUTOBAUD_TIMER_LOCATION    TIMER_ROUTE_LOCATION_LOC4
#define AUTOBAUD_TIMER_INT_MASK    TIMER_IFC_CC0
#define AUTOBAUD_TIMER_ROUTE       TIMER_ROUTE_CC0PEN

#else
#define AUTOBAUD_TIMER_CHANNEL     1
#define AUTOBAUD_TIMER_LOCATION    TIMER_ROUTE_LOCATION_LOC1
#define AUTOBAUD_TIMER_INT_MASK    TIMER_IFC_CC1
#define AUTOBAUD_TIMER_ROUTE       TIMER_ROUTE_CC1PEN
#endif
#endif

/******************************************************************************
 ** Check if we must enter bootloader commandline loop.                      **
 ******************************************************************************/
#define SWCLK_PIN_IS_HI()        ( ( GPIO->P[5].DIN & 0x1 ) == 0x1 )
#define SWCLK_PIN_IS_LO()        ( ( GPIO->P[5].DIN & 0x1 ) == 0x0 )

/******************************************************************************
 ** The size of the bootloader flash image                                   **
 ******************************************************************************/
#define BOOTLOADER_SIZE           (16*1024)       /* 16 KB */

/******************************************************************************
 ** The maximum flash size of any EFM32 part                                 **
 ******************************************************************************/
#define MAX_SIZE_OF_FLASH         (1024*1024)     /* 1 MB */

/******************************************************************************
 ** The size of a mass erase block                                           **
 ******************************************************************************/
#if defined( _EFM32_HAPPY_FAMILY )
#define MASSERASE_BLOCK_SIZE      (64*1024)       /* 64 KB */
#else
#define MASSERASE_BLOCK_SIZE      (512*1024)      /* 512 KB */
#endif

/******************************************************************************
 ** This function sets up GPIO for the USART used in the bootloader.         **
 ******************************************************************************/
#if defined( _EZR32_LEOPARD_FAMILY )
__STATIC_INLINE void CONFIG_UsartGpioSetup(void)
{
  /* Use USART1 location 2
   * 0 : TX - Pin D7, RX - Pin D6
   * Configure GPIO pins LOCATION 2 as push pull (TX) and input (RX)
   * To avoid false start, configure output as high
   */
  GPIO->P[3].DOUT = (1 << 7);
  GPIO->P[3].MODEL = GPIO_P_MODEL_MODE7_PUSHPULL | GPIO_P_MODEL_MODE6_INPUT;
}

#elif defined( _EFM32_HAPPY_FAMILY )
__STATIC_INLINE void CONFIG_UsartGpioSetup(void)
{
   /*
   * LEUART0, location 3:
   * TX: F0, RX: F1
   */
  GPIO->P[5].MODEL = GPIO_P_MODEL_MODE0_PUSHPULL | GPIO_P_MODEL_MODE1_INPUT;
}

#else /* GIANT/LEOPARD/WONDER */
__STATIC_INLINE void CONFIG_UsartGpioSetup(void)
{
  /* Use USART0 location 0
   * 0 : TX - Pin E10, RX - Pin E11
   * Configure GPIO pins LOCATION 1 as push pull (TX)
   * and input (RX)
   * To avoid false start, configure output as high
   */
  GPIO->P[4].DOUT = (1 << 10);
  GPIO->P[4].MODEH = GPIO_P_MODEH_MODE10_PUSHPULL | GPIO_P_MODEH_MODE11_INPUT;
}
#endif

#endif
