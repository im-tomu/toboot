/**************************************************************************//**
 * @file loader.c
 * @brief USB/USART0 bootloader 1. level loader.
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
#include <string.h>
#include <stdbool.h>
#include "em_device.h"
#include "boot.h"

const
#if defined( EFM32GG990F1024 )
#include "loader-giant.h"
#elif defined( EFM32LG990F256 )
#include "loader-leopard.h"
#elif defined( _EZR32_LEOPARD_FAMILY )
#include "loader-ezrleopard.h"
#elif defined( _EFM32_WONDER_FAMILY )
#include "loader-wonder.h"
#endif

/**************************************************************************//**
 * The main entry point.
 *****************************************************************************/
int main(void)
{
  __set_MSP( ( 0x20000000 + sizeof( bootloader ) + 0x400 ) & 0xFFFFFFF0 );

  /* Load the entire bootloader into SRAM. */
  memcpy( (void*)0x20000000, bootloader, sizeof( bootloader ) );

  /* Start executing the bootloader. */
  BOOT_jump( *(uint32_t*)0x20000000, *(uint32_t*)0x20000004 );
}
