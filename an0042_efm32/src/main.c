/**************************************************************************//**
 * @file main.c
 * @brief USB/USART0 bootloader.
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
#include "em_emu.h"
#include "em_usb.h"
#include "config.h"
#include "cdc.h"
#include "crc.h"
#include "flash.h"
#include "boot.h"
#include "autobaud.h"
#include "xmodem.h"
#include "bootldio.h"
#include "retargetdebug.h"

/*** Typedef's and defines. ***/
#define BOOTLOADER_VERSION_STRING "EFM32HG bootloader v1.0"

/** Version string, used when the user connects */
#if !defined( BOOTLOADER_VERSION_STRING )
#error "No bootloader version string defined !"
#endif

#define USER_PAGE_START USERDATA_BASE
#define USER_PAGE_END   (USERDATA_BASE + 0x200)
#define LOCK_PAGE_START LOCKBITS_BASE
#define LOCK_PAGE_END   (LOCKBITS_BASE + 0x200)

#define DEBUG_LOCK_WORD (LOCKBITS_BASE + (127 * 4))

/*** Function prototypes. ***/

__ramfunc __noreturn static void commandlineLoop(  void );
__ramfunc static void verify( uint32_t start, uint32_t end );
__ramfunc static void Disconnect( int predelay, int postdelay );
static void StartRTC( void );

/*** The descriptors for a USB CDC device. ***/
#include "descriptors.h"

/*** Variables ***/

#if !defined( NO_RAMFUNCS )
/* Vector table in RAM. We construct a new vector table to conserve space in
 * flash as it is sparsly populated. */
#pragma location=0x20000000
__no_init uint32_t vectorTable[47];
#endif

/*
 * This variable holds the computed CRC-16 of the bootloader and is used during
 * production testing to ensure the correct programming of the bootloader.
 * This can safely be omitted if you are rolling your own bootloader.
 * It is placed rigth after the interrupt vector table.
 */
#pragma location=0x200000dc
__no_init uint32_t bootloaderCRC;

/**************************************************************************//**
 * The main entry point.
 *****************************************************************************/
int main(void)
{
  int msElapsed, i;

#if defined( NO_RAMFUNCS )
  /* Set new vector table pointer */
  SCB->VTOR = 0x20000000;
#endif

  /* Enable peripheral clocks. */
  CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
  CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO | BOOTLOADER_USART_CLOCKEN
                     | AUTOBAUD_TIMER_CLOCK ;

  /* Enable LE and DMA interfaces. */
  CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE | CMU_HFCORECLKEN0_DMA;

#if defined( BL_DEBUG )
  RETARGET_SerialInit();        /* Setup debug serialport etc. */
  USB_PUTS( "EFM/EZR/EFR USB/UART bootloader\r\n" );
#endif

  /* Calculate CRC16 for the bootloader itself and the Device Information page. */
  /* This is used for production testing and can safely be omitted in */
  /* your own code. */
  bootloaderCRC  = CRC_calc((void *) 0x0, (void *) BOOTLOADER_SIZE);
  bootloaderCRC |= CRC_calc((void *) 0x0FE081B2, (void *) 0x0FE08200) << 16;

  /* Setup LFA/LFB clock sources. */
#if defined( BOOTLOADER_LEUART_CLOCKEN )
  CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;
#else
  CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO;
#endif

  StartRTC();

#if !defined( SIMULATE_SWCLK_PIN_HI )
  while ( SWCLK_PIN_IS_LO() )
  {
    USB_PUTS( "SWCLK is low\r\n" );

    if ( BOOT_checkFirmwareIsValid() )
    {
      USB_PUTS( "Booting application\r\n  " );
      BOOT_boot();
    }
    else
    {
      USB_PUTS( "No valid application, resetting EFM32... \r\n" );

      /* Go to EM2 and wait for RTC wakeup. */
      EMU_EnterEM2( false );
    }
  }
#endif

#if defined( BOOTLOADER_LEUART_CLOCKEN )
  /* Enable LEUART clock. */
  CMU->LFBCLKEN0 = BOOTLOADER_LEUART_CLOCKEN;
#endif

  NVIC_DisableIRQ( RTC_IRQn );

#if !defined( CMU_OSCENCMD_USHFRCOEN )
  /* Try to start HFXO. */

  CMU_OscillatorEnable( cmuOsc_HFXO, true, false );

  /* Wait approx. 1 second to see if HFXO starts. */
  i = 1500000;
  while ( i && !( CMU->STATUS & CMU_STATUS_HFXORDY ) )
  {
    i--;
  }
#else
  i = 1;
#endif
  
  USBTIMER_Init();

#if !defined( NO_RAMFUNCS )
  // Build new IRQ vector table, only init the IRQ's we actually need.
  vectorTable[ RTC_IRQn + 16] = (uint32_t) RTC_IRQHandler;
  vectorTable[ USB_IRQn + 16] = (uint32_t) USB_IRQHandler;
  vectorTable[ AUTOBAUD_TIMER_IRQn + 16] = (uint32_t) AUTOBAUD_TIMER_IRQHANDLER;
#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
  vectorTable[ GPIO_EVEN_IRQn + 16] = (uint32_t) GPIO_IRQHandler;
#endif
  // Set new vector table pointer.
  SCB->VTOR = 0x20000000;
#endif

  if ( i == 0 )
  {
#if defined( _CMU_HFRCOCTRL_BAND_28MHZ )
    CMU_HFRCOBandSet( cmuHFRCOBand_28MHz );
#else
    CMU_HFRCOBandSet( cmuHFRCOBand_21MHz );
#endif
  }
  else
  {
#if !defined( CMU_OSCENCMD_USHFRCOEN )
    CMU_ClockSelectSet( cmuClock_HF, cmuSelect_HFXO );
#endif
    USBD_Init( &initstruct );       /* Start USB CDC functionality  */
  }

  AUTOBAUD_start();                 /* Start autobaud               */
  msElapsed = 0;
  while ( msElapsed < 30000 ) /* Wait max 30 secs for UART or USB connection */
  {
    if ( AUTOBAUD_completed() )
      break;

    if ( CDC_Configured )
    {
      BOOTLDIO_setMode( CDC_Configured );
      break;
    }

    USBTIMER_DelayMs( 100 );
    msElapsed += 100;
  }
  AUTOBAUD_stop();

  if ( msElapsed >= 30000 )
  {
    USB_PUTS( "USART0/USB timeout, resetting EFM32...\r\n  " );
    Disconnect( 0, 2000 );
    SCB->AIRCR = 0x05FA0004;        /* Reset EFM32. */
  }

  /* Print a message to show that we are in bootloader mode */

  BOOTLDIO_printString("\r\n\r\nBOOTLOADER version " BOOTLOADER_VERSION_STRING ", Chip ID " );

  /* Print the chip ID. This is useful for production tracking */
  BOOTLDIO_printHex(DEVINFO->UNIQUEH);
  BOOTLDIO_printHex(DEVINFO->UNIQUEL);
  BOOTLDIO_printString("\r\n");

  /* Figure out correct flash geometry. */
  FLASH_CalcFlashSize();
  /* Initialize flash for writing */
  FLASH_init();

  /* Start executing command line */
  commandlineLoop();
}


/**************************************************************************//**
 * @brief
 *   The main command line loop. Placed in Ram so that it can still run after
 *   a destructive write operation.
 *   NOTE: __ramfunc is a IAR specific instruction to put code into RAM.
 *   This allows the bootloader to survive a destructive upload.
 *****************************************************************************/
__ramfunc __noreturn static void commandlineLoop( void )
{
  uint8_t  c;
  static uint8_t readyString[]   = "\r\nReady\r\n";
  static uint8_t okString[]      = "\r\nOK\r\n";
  static uint8_t unknownString[] = "\r\n?\r\n";
  static uint8_t failString[]    = "\r\nFail\r\n";
  static uint8_t welcomeString[] = "\r\nBOOTLOADER version " BOOTLOADER_VERSION_STRING ", Chip ID ";
  static uint8_t newLineString[] = "\r\n";

  /* The main command loop */
  while (1)
  {
    /* Retrieve new character */
    c = BOOTLDIO_rxByte();
    /* Echo */
    if (c != 0)
    {
      BOOTLDIO_txByte( c );

      switch (c)
      {
      /* Bootloader version command */
      case 'i':
        /* Print version */
        BOOTLDIO_printString( welcomeString );

        /* Print the chip ID */
        BOOTLDIO_printHex( DEVINFO->UNIQUEH );
        BOOTLDIO_printHex( DEVINFO->UNIQUEL );
        BOOTLDIO_printString( newLineString );
        break;

      /* Upload command */
      case 'u':
        BOOTLDIO_printString( readyString );
        XMODEM_download( BOOTLOADER_SIZE, flashSize );
        break;

      /* Destructive upload command */
      case 'd':
        BOOTLDIO_printString( readyString );
        XMODEM_download( 0, flashSize );
        break;

      /* Write to user page */
      case 't':
        BOOTLDIO_printString( readyString );
        XMODEM_download( USER_PAGE_START, USER_PAGE_END );
        break;

      /* Write to lock bits */
      case 'p':
        BOOTLDIO_printString( readyString );
        XMODEM_download( LOCK_PAGE_START, LOCK_PAGE_END );
        break;

      /* Boot into new program */
      case 'b':
        Disconnect( 5000, 2000 );
        BOOT_boot();
        break;

      /* Debug lock */
      case 'l':
#if defined( BL_DEBUG )
        /* We check if there is a debug session active in DHCSR. If there is we
         * abort the locking. This is because we wish to make sure that the debug
         * lock functionality works without a debugger attatched. */
        if ((CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) != 0x0)
        {
          USB_PUTS( "\r\n\r\n **** WARNING: DEBUG SESSION ACTIVE. NOT LOCKING!  **** \r\n\r\n" );
          BOOTLDIO_printString( "Debug active.\r\n" );
        }
        else
        {
          USB_PUTS( "Starting debug lock sequence.\r\n" );
#endif
          FLASH_writeWord( DEBUG_LOCK_WORD, 0x0 );
          if ( *(volatile uint32_t*)DEBUG_LOCK_WORD == 0x0 )
          {
            BOOTLDIO_printString( okString );
          }
          else
          {
            BOOTLDIO_printString( failString );
          }
#if defined( BL_DEBUG )
          USB_PRINTF( "Debug lock word: 0x%x \r\n", *((uint32_t *) DEBUG_LOCK_WORD) );
        }
#endif
        break;

      /* Verify content by calculating CRC of entire flash */
      case 'v':
        verify( 0, flashSize );
        break;

      /* Verify content by calculating CRC of application area */
      case 'c':
        verify( BOOTLOADER_SIZE, flashSize );
        break;

      /* Verify content by calculating CRC of user page.*/
      case 'n':
        verify( USER_PAGE_START, USER_PAGE_END );
        break;

      /* Verify content by calculating CRC of lock page */
      case 'm':
        verify( LOCK_PAGE_START, LOCK_PAGE_END );
        break;

      /* Reset command */
      case 'r':
        Disconnect( 5000, 2000 );

        /* Write to the Application Interrupt/Reset Command Register to reset
         * the EFM32. See section 9.3.7 in the reference manual. */
        SCB->AIRCR = 0x05FA0004;
        break;

      default:
        BOOTLDIO_printString( unknownString );
      }
    }
  }
}


/**************************************************************************//**
 * @brief
 *   Helper function to print flash write verification using CRC
 * @param start
 *   The start of the block to calculate CRC of.
 * @param end
 *   The end of the block. This byte is not included in the checksum.
 *****************************************************************************/
__ramfunc static void verify(uint32_t start, uint32_t end)
{
  static uint8_t crcString[]     = "\r\nCRC: ";
  static uint8_t newLineString[] = "\r\n";

  BOOTLDIO_printString(crcString);
  BOOTLDIO_printHex(CRC_calc((void *) start, (void *) end));
  BOOTLDIO_printString(newLineString);
}


/**************************************************************************//**
 * Disconnect USB link with optional delays.
 *****************************************************************************/
__ramfunc static void Disconnect( int predelay, int postdelay )
{
  if ( BOOTLDIO_usbMode() )
  {
    /* Allow time to do a disconnect in a terminal program. */
    USBTIMER_DelayMs( predelay );

    USBD_Disconnect();

    /*
     * Stay disconnected long enough to let host OS tear down the
     * USB CDC driver.
     */
    USBTIMER_DelayMs( postdelay );
  }
}

/**************************************************************************//**
 * @brief RTC IRQ Handler
 *****************************************************************************/
__root void RTC_IRQHandler( void )
{
  /* Clear interrupt flag */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
}

/**************************************************************************//**
 * Initialize and start RTC.
 *****************************************************************************/
static void StartRTC( void )
{
  /* Enable LFRCO for RTC */
  CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
  /* Enable RTC */
  CMU->LFACLKEN0 = CMU_LFACLKEN0_RTC;

  /* Clear interrupt flags */
  RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
  /* 250 ms wakeup time */
  RTC->COMP0 = ( PIN_LOOP_INTERVAL * SystemLFRCOClockGet() ) / 1000;
  /* Enable Interrupts on COMP0 */
  RTC->IEN = RTC_IEN_COMP0;
  /* Enable RTC interrupts */
  NVIC_EnableIRQ(RTC_IRQn);
  /* Enable RTC */
  RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_DEBUGRUN | RTC_CTRL_EN;
}
