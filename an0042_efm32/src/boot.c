/**************************************************************************//**
 * @file boot.c
 * @brief Boot Loader
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

#include <stdbool.h>

#include "em_device.h"
#include "boot.h"
#include "em_usb.h"

/**************************************************************************//**
 * @brief Checks to see if the reset vector of the application is valid
 * @return false if the firmware is not valid, true if it is.
 *****************************************************************************/
bool BOOT_checkFirmwareIsValid(void)
{
  uint32_t pc;

  pc = *((uint32_t *) BOOTLOADER_SIZE + 1);

  if (pc < MAX_SIZE_OF_FLASH)
    return true;

  return false;
}

/**************************************************************************//**
 * @brief This function sets up the Cortex M-3 with a new SP and PC.
 *****************************************************************************/
__ramfunc void BOOT_jump(uint32_t sp, uint32_t pc)
{
  (void) sp;
  (void) pc;

  /* Set new MSP, PSP based on SP (r0)*/
  __asm("msr msp, r0");
  __asm("msr psp, r0");

  /* Jump to PC (r1)*/
  __asm("mov pc, r1");
}

/**************************************************************************//**
 * @brief Boots the application
 *****************************************************************************/
__ramfunc void BOOT_boot(void)
{
  uint32_t pc, sp;

  /* Reset all used registers to their default value. */

  /* Disable all interrupts. */
  NVIC->ICER[0] = 0xFFFFFFFF;
#if ( __CORTEX_M != 0 )
  NVIC->ICER[1] = 0xFFFFFFFF;
#endif

  /* Disable USB */
  USBD_Stop();

  /* Reset memory system controller settings. */
  MSC->READCTRL  = _MSC_READCTRL_RESETVALUE;
  MSC->WRITECTRL = _MSC_WRITECTRL_RESETVALUE;
  MSC->LOCK = 0;

  /* Reset GPIO settings. */
#ifdef USART_OVERLAPS_WITH_BOOTLOADER
  CMU->LFBCLKEN0    = _CMU_LFBCLKEN0_RESETVALUE;
  GPIO->EXTIPSELL   = _GPIO_EXTIPSELL_RESETVALUE;
  GPIO->EXTIFALL    = _GPIO_EXTIFALL_RESETVALUE;
  GPIO->IEN         = _GPIO_IEN_RESETVALUE;
  GPIO->IFC         = 0xFFFFFFFF;
#endif
  GPIO->ROUTE = _GPIO_ROUTE_RESETVALUE;
#if defined( _EZR32_LEOPARD_FAMILY )
  GPIO->P[3].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[3].DOUT  = _GPIO_P_DOUT_RESETVALUE;
#elif defined( _EFM32_HAPPY_FAMILY )
  GPIO->P[5].MODEL = _GPIO_P_MODEL_RESETVALUE;
  GPIO->P[5].DOUT  = _GPIO_P_DOUT_RESETVALUE;
#else
  GPIO->P[4].MODEH = _GPIO_P_MODEH_RESETVALUE;
  GPIO->P[4].DOUT  = _GPIO_P_DOUT_RESETVALUE;
#endif

  /* Reset DMA controller settings. */
  DMA->CONFIG     = _DMA_CONFIG_RESETVALUE;
  DMA->CTRLBASE   = _DMA_CTRLBASE_RESETVALUE;
  DMA->CH[0].CTRL = _DMA_CH_CTRL_RESETVALUE;
  DMA->CHENC      = 0xFFFFFFFF;

  /* Reset TIMER0 settings. */
  TIMER0->CMD        = TIMER_CMD_STOP;
  TIMER0->TOP        = _TIMER_TOP_RESETVALUE;
  TIMER0->CTRL       = _TIMER_CTRL_RESETVALUE;
  TIMER0->CC[0].CTRL = _TIMER_CC_CTRL_RESETVALUE;

  /* Reset TIMER1 settings. */
  AUTOBAUD_TIMER->CMD        = TIMER_CMD_STOP;
  AUTOBAUD_TIMER->TOP        = _TIMER_TOP_RESETVALUE;
  AUTOBAUD_TIMER->IEN        = _TIMER_IEN_RESETVALUE;
  AUTOBAUD_TIMER->IFC        = 0xFFFFFFFF;
#if ( AUTOBAUD_TIMER_CHANNEL == 1 )
  AUTOBAUD_TIMER->CC[1].CTRL = _TIMER_CC_CTRL_RESETVALUE;
#endif

  /* Reset RTC settings. */
  RTC->IEN    = _RTC_IEN_RESETVALUE;
  RTC->COMP0  = _RTC_COMP0_RESETVALUE;
  RTC->CTRL   = _RTC_CTRL_RESETVALUE;

  /* Reset UART settings. */
  BOOTLOADER_USART->ROUTE  = _USART_ROUTE_RESETVALUE;
  BOOTLOADER_USART->CLKDIV = _USART_CLKDIV_RESETVALUE;
  BOOTLOADER_USART->CMD    = USART_CMD_RXDIS | USART_CMD_TXDIS;

  /* Wait for LF peripheral syncronization. */
  while (RTC->SYNCBUSY & _RTC_SYNCBUSY_MASK);
  while (CMU->SYNCBUSY & CMU_SYNCBUSY_LFACLKEN0);

  /* Switch to default cpu clock. */
  CMU->CMD      = CMU_CMD_HFCLKSEL_HFRCO;
  CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS | CMU_OSCENCMD_LFRCODIS;

  /* Reset clock registers used. */
  CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
  CMU->HFPERCLKDIV  = _CMU_HFPERCLKDIV_RESETVALUE;
  CMU->HFPERCLKEN0  = _CMU_HFPERCLKEN0_RESETVALUE;
  CMU->LFCLKSEL     = _CMU_LFCLKSEL_RESETVALUE;
  CMU->LFACLKEN0    = _CMU_LFACLKEN0_RESETVALUE;

  /* Set new vector table pointer */
  SCB->VTOR = (uint32_t)BOOTLOADER_SIZE;

  /* Read new SP and PC from vector table */
  sp = *((uint32_t *)BOOTLOADER_SIZE    );
  pc = *((uint32_t *)BOOTLOADER_SIZE + 1);

  BOOT_jump(sp, pc);
}
