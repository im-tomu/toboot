/***************************************************************************//**
 * @file retargetdebug.c
 * @brief Provide stdio retargeting to UART or USB CDC for debugging purposes.
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

#include <stdio.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "config.h"
#include "retargetdebug.h"

#if defined( BL_DEBUG )

#define RETARGET_IRQ_NAME  USART0_RX_IRQHandler
#define RETARGET_CLK       cmuClock_USART0
#define RETARGET_IRQn      USART0_RX_IRQn
#define RETARGET_UART      USART0
#define RETARGET_TX        USART_Tx
#define RETARGET_RX        USART_Rx
#define RETARGET_LOCATION  USART_ROUTE_LOCATION_LOC0
#define RETARGET_TXPORT    gpioPortE
#define RETARGET_TXPIN     10
#define RETARGET_RXPORT    gpioPortE
#define RETARGET_RXPIN     11

/**************************************************************************//**
 * @brief Intializes UART/LEUART
 *****************************************************************************/
void RETARGET_SerialInit(void)
{
  /* Configure GPIO pins */
  CMU_ClockEnable(cmuClock_GPIO, true);
  /* To avoid false start, configure output as high */
  GPIO_PinModeSet(RETARGET_TXPORT, RETARGET_TXPIN, gpioModePushPull, 1);
  //GPIO_PinModeSet(RETARGET_RXPORT, RETARGET_RXPIN, gpioModeInput, 0);

  USART_TypeDef *usart = RETARGET_UART;
  USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;

  /* Enable peripheral clocks */
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(RETARGET_CLK, true);

  /* Configure USART for basic async operation */
  init.enable = usartDisable;
  USART_InitAsync(usart, &init);

  /* Enable pins at correct location */
  usart->ROUTE = /*USART_ROUTE_RXPEN |*/ USART_ROUTE_TXPEN | RETARGET_LOCATION;

  /* Finally enable it */
  USART_Enable(usart, usartEnable);

#if !defined( __CROSSWORKS_ARM ) && defined( __GNUC__ )
  setvbuf( stdout, NULL, _IONBF, 0 ); /*Set unbuffered mode for stdout (newlib)*/
#endif
}


/**************************************************************************//**
 * @brief Receive a byte from USART/LEUART and put into global buffer
 * @return -1 on failure, or positive character integer on sucesss
 *****************************************************************************/
int RETARGET_ReadChar(void)
{
  int c = -1;

  return c;
}

/**************************************************************************//**
 * @brief Transmit single byte to USART/LEUART
 * @param data Character to transmit
 *****************************************************************************/
int RETARGET_WriteChar(char c)
{
  RETARGET_TX(RETARGET_UART, c);
  return c;
}
#endif
