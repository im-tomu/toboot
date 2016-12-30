/**************************************************************************//**
 * @file autobaud.c
 * @brief Bootloader autobaud functions.
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
#include "em_device.h"
#include "em_cmu.h"
#include "em_usb.h"
#include "config.h"
#include "xmodem.h"
#include "bootldio.h"
#include "autobaud.h"

#if defined( _EFM32_HAPPY_FAMILY )
#define SAMPLE_MAX 5    // We get one less input capture event on Happy Geckos
#else
#define SAMPLE_MAX 6
#endif

static volatile int currentSample = 0;
static volatile uint16_t samples[ SAMPLE_MAX ];

#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
/**************************************************************************//**
 * @brief
 *   GPIO interrupt handler. The purpose of this is to detect if there is a
 *   debugger attatched (Activity on SWCLK - F0). If there is we should yield
 *   and let the debugger access the pin.
 * @note
 *   This is only required if the bootloader overlaps with the debug pins.
 *****************************************************************************/
__root void GPIO_IRQHandler(void)
{
  /* Disable all interrupts */
  __disable_irq();

  USB_PUTS( "Activity on SWCLK. Suspending.\r\n" );

  /* Yield control to debugger. */
  AUTOBAUD_TIMER->ROUTE = _TIMER_ROUTE_RESETVALUE;
  GPIO->ROUTE           = _GPIO_ROUTE_RESETVALUE;
  for(;;);
}
#endif

/**************************************************************************//**
 * @brief
 *   AUTOBAUD_TIMER interrupt handler. This function stores the current value of the
 *   timer in the samples array.
 *****************************************************************************/
__root void AUTOBAUD_TIMER_IRQHANDLER( void )
{
  uint32_t period;

  /* Clear CC flag */
  AUTOBAUD_TIMER->IFC = AUTOBAUD_TIMER_INT_MASK;

  /* Store CC value in samples array */
  if ( currentSample < SAMPLE_MAX )
  {
    period = AUTOBAUD_TIMER->CC[AUTOBAUD_TIMER_CHANNEL].CCV;
    samples[ currentSample++ ] = period;
  }
}

/**************************************************************************//**
 * @brief
 *   This function uses the samples array to estimate the baudrate.
 *   Assumes that the TIMER is not prescaled.
 *
 * @return
 *   Measured baudrate.
 *****************************************************************************/
static uint32_t EstimateBaudRate( void )
{
  int i;
  uint16_t timeDiff;
  uint32_t periodSum = 0;

  /*
   * Calculate the periods. Discard the first (false) sample value.
   * Accumulate period values.
   */
  for (i = 2; i < currentSample; i++)
  {
    timeDiff   = samples[ i ];
    timeDiff  -= samples[ i-1 ];
    periodSum += timeDiff;
  }

  return ( CMU_ClockFreqGet( cmuClock_HFPER ) * 2
           * (currentSample - 2) ) / periodSum;
}

/**************************************************************************//**
 * @brief
 *  This function sets up AUTOBAUD_TIMER to estimate the needed CLKDIV needed for
 *  BOOTLOADER_USART. It does this by using compare channel
 *  AUTOBAUD_TIMER_CHANNEL and registering how many HF clock cycles occur
 *  between rising edges.
 *
 *  This assumes that AUTOBAUD_TIMER AUTOBAUD_TIMER_CHANNEL overlaps with the
 *  BOOTLOADER_USART RX pin.
 *****************************************************************************/
void AUTOBAUD_start( void )
{
  USB_PUTS( "Starting autobaud.\r\n" );

  /* Setup pins for USART */
  CONFIG_UsartGpioSetup();

#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
  /* Set up GPIO interrupts on falling edge on the SWCLK. */
  /* If we see such an event, then there is a debugger attatched. */
  /* SWCLK is port F0 */
  GPIO->EXTIPSELL = GPIO_EXTIPSELL_EXTIPSEL0_PORTF;
  GPIO->EXTIFALL = 1;
  GPIO->IFC = 1;
  GPIO->IEN = 1;
#endif

  /* Set a high top value to avoid overflow */
  AUTOBAUD_TIMER->TOP = UINT32_MAX;

  /* Set up compare channel. Trigger on rising edge and capture value. */
  AUTOBAUD_TIMER->CC[AUTOBAUD_TIMER_CHANNEL].CTRL =
                TIMER_CC_CTRL_MODE_INPUTCAPTURE | TIMER_CC_CTRL_ICEDGE_RISING;

  /* Set up AUTOBAUD_TIMER to location AUTOBAUD_TIMER_LOCATION */
  AUTOBAUD_TIMER->ROUTE = AUTOBAUD_TIMER_LOCATION | AUTOBAUD_TIMER_ROUTE;

  /* Clear all timer interrupt flags */
  AUTOBAUD_TIMER->IFC = 0xFFFFFFFF;

  /* Enable interrupt on channel capture */
  AUTOBAUD_TIMER->IEN = AUTOBAUD_TIMER_INT_MASK;

  /* Enable interrupts */
  NVIC_EnableIRQ( AUTOBAUD_TIMER_IRQn );
#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
  NVIC_EnableIRQ( GPIO_EVEN_IRQn );
#endif

  /* Start the timer */
  AUTOBAUD_TIMER->CMD = TIMER_CMD_START;
}


/**************************************************************************//**
 * @brief
 *  This function stops AUTOBAUD_TIMER.
 *****************************************************************************/
void AUTOBAUD_stop( void )
{
  /* Disable interrupts in Cortex */
  NVIC_DisableIRQ( AUTOBAUD_TIMER_IRQn );
#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
  NVIC_DisableIRQ( GPIO_EVEN_IRQn );
#endif

  /* Disable routing of TIMER. */
  AUTOBAUD_TIMER->ROUTE = _TIMER_ROUTE_RESETVALUE;
}

/**************************************************************************//**
 * @brief
 *  This function checks for autobaud completion.
 *
 * @return
 *   true on succesful autobaud completion.
 *   false if autobaud not yet completed.
 *****************************************************************************/
bool AUTOBAUD_completed( void )
{
  uint32_t baudRate;
  uint32_t clkdiv;


  if ( currentSample < SAMPLE_MAX )
    return false;

  AUTOBAUD_stop();

  baudRate = EstimateBaudRate();

#if defined( BOOTLOADER_LEUART_CLOCKEN )
  clkdiv  = (32 * CMU_ClockFreqGet( BOOTLOADER_USART_CLOCK )) + (baudRate/2);
  clkdiv /= baudRate;
  clkdiv -= 32;
  clkdiv *= 8;
#else
  clkdiv  = 4 * CMU_ClockFreqGet( cmuClock_HFPER ); // Add (16*bautrate)/2 ?
  clkdiv /= 16 * baudRate;
  clkdiv -= 4;
  clkdiv *= 64;
#endif

  /* Initialize the USART */
  BOOTLDIO_usartInit( clkdiv );

  USB_PUTS(   "Autobaud complete.\r\n" );
  USB_PRINTF( "Measured baudrate is %d\r\n", baudRate );
  USB_PRINTF( "New USART clkdiv is %d\r\n  ", clkdiv );

  return true;
}
