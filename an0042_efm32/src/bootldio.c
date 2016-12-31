/**************************************************************************//**
 * @file bootldio.c
 * @brief IO code, USART or USB, for the EFM32 bootloader
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
#include "em_usb.h"
#include "xmodem.h"
#include "bootldio.h"
#include "config.h"

#define USB_BUF_SIZ sizeof(XMODEM_packet)

static bool useUsb = false;

STATIC_UBUF( usbBuffer, USB_BUF_SIZ );

static uint32_t           usbXferCnt;
static volatile bool      usbXferDone;
static USB_Status_TypeDef usbXferStatus;

/**************************************************************************//**
 * @brief strlen() reimplementation.
 *****************************************************************************/
__ramfunc __STATIC_INLINE size_t slStrLen(const char * str)
{
   const char * ptr;
   for (ptr = str; *ptr; ++ptr)
     ;
   return ptr - str;
}

/**************************************************************************//**
 * @brief memcpy() reimplementation.
 *****************************************************************************/
__ramfunc __STATIC_INLINE void *slMemCpy(void *dest, const void *source, size_t n)
{
  const char *src = source;
  char *dst = dest;

  while (n--)
  {
    *dst++ = *src++;
  }

  return dest;
}

/**************************************************************************//**
 * @brief
 *    Callback function called whenever a packet with data has been
 *    transferred on USB.
 *****************************************************************************/
__ramfunc static int UsbDataXferred( USB_Status_TypeDef status,
                           uint32_t xferred, uint32_t remaining )
{
  (void)remaining;            /* Unused parameter */

  usbXferStatus = status;
  usbXferCnt    = xferred;
  usbXferDone   = true;

  return USB_STATUS_OK;
}

/***************************************************************************//**
 * @brief
 *   Set I/O in correct mode.
 *
 * @param[in] usbMode
 *   True if USB shall be used, USART otherwise.
 ******************************************************************************/
void BOOTLDIO_setMode( bool usb )
{
  useUsb = usb;
}

/***************************************************************************//**
 * @brief
 *   Get current I/O mode.
 *
 * @return
 *   True if USB is current I/O mode, false if USART.
 ******************************************************************************/
__ramfunc bool BOOTLDIO_usbMode( void )
{
  return useUsb;
}

/***************************************************************************//**
 * @brief
 *   Prints an int in hex.
 *
 * @param integer
 *   The integer to be printed.
 ******************************************************************************/
__ramfunc void BOOTLDIO_printHex( uint32_t integer )
{
  uint8_t c;
  int     i, j, digit;

  for ( i = 7, j = 0; i >= 0; i--, j++ )
  {
    digit = (integer >> (i * 4)) & 0xf;
    if (digit < 10)
    {
      c = digit + 0x30;
    }
    else
    {
      c = digit + 0x37;
    }
    if ( useUsb )
    {
      usbBuffer[ j ] = c;
    }
    else
    {
      BOOTLDIO_txByte(c);
    }
  }

  if ( useUsb )
  {
    usbBuffer[ j ] = '\0';
    BOOTLDIO_printString( usbBuffer );
  }
}

/**************************************************************************//**
 * @brief Get single byte from USART or USB
 *****************************************************************************/
__ramfunc uint8_t BOOTLDIO_rxByte( void )
{
  uint8_t  retVal;
  uint32_t timer = 2000000;

  if ( useUsb )
  {
    usbXferDone = false;
    USBD_Read( EP_DATA_OUT, usbBuffer, USB_BUF_SIZ, UsbDataXferred );
    while ( !usbXferDone ){}
    retVal = usbXferStatus == USB_STATUS_OK ? usbBuffer[0] : 0;
  }
  else
  {
#if defined( BOOTLOADER_LEUART_CLOCKEN )
    while (!(BOOTLOADER_USART->STATUS & LEUART_STATUS_RXDATAV) && --timer ){}
#else
    while (!(BOOTLOADER_USART->STATUS & USART_STATUS_RXDATAV) && --timer ){}
#endif

    retVal = timer > 0 ? (uint8_t)BOOTLOADER_USART->RXDATA : 0;
  }

  return retVal;
}

/**************************************************************************//**
 * @brief Get an XMODEM packet from USB with optional timeout.
 *
 * @param[in] p
 *   Pointer to XMODEM storage space.
 *
 * @param[in] timeout
 *   Transmission timeout in milliseconds, no timeout if zero.
 *
 * @return
 *   True if a transmission took place.
 *****************************************************************************/
__ramfunc bool BOOTLDIO_getPacket( XMODEM_packet *p, int timeout )
{
  usbXferDone = false;
  USBD_Read( EP_DATA_OUT, usbBuffer, USB_BUF_SIZ, UsbDataXferred );

  if ( timeout )
  {
    while ( !usbXferDone && --timeout )
    {
      USBTIMER_DelayMs( 1 );
    }

    if ( timeout <= 0 )
    {
      USBD_AbortTransfer( EP_DATA_OUT );
      return false;
    }
  }
  else
  {
    while ( !usbXferDone ){}
  }

  /*
   * Copy data at p+1 so that data payload becomes even aligned.
   * See definition of XMODEM_packet.
   */
  slMemCpy( (uint8_t*)p + 1, usbBuffer, usbXferCnt );
  return true;
}

/**************************************************************************//**
 * @brief Transmit single byte to USART or USB
 *****************************************************************************/
__ramfunc void BOOTLDIO_txByte( uint8_t data )
{
  if ( useUsb )
  {
    usbBuffer[ 0 ] = data;
    usbXferDone = false;
    USBD_Write( EP_DATA_IN, usbBuffer, 1, UsbDataXferred );
    while ( !usbXferDone ){}
  }
  else
  {
    /* Check that transmit buffer is empty */
#if defined( BOOTLOADER_LEUART_CLOCKEN )
    while (!(BOOTLOADER_USART->STATUS & LEUART_STATUS_TXBL)){}
#else
    while (!(BOOTLOADER_USART->STATUS & USART_STATUS_TXBL)){}
#endif

    BOOTLOADER_USART->TXDATA = (uint32_t) data;
  }
}

/**************************************************************************//**
 * @brief Transmit null-terminated string to USART or USB
 *****************************************************************************/
__ramfunc void BOOTLDIO_printString( const uint8_t *string )
{
  int len;

  if ( useUsb )
  {
    len = slStrLen( (char*)string );
    slMemCpy( usbBuffer, string, len );
    usbXferDone = false;
    USBD_Write( EP_DATA_IN, usbBuffer, len, UsbDataXferred );
    while ( !usbXferDone ){}
  }
  else
  {
    while (*string != 0)
    {
      BOOTLDIO_txByte(*string++);
    }
  }
}

/**************************************************************************//**
 * @brief Intializes BOOTLOADER_USART
 *
 * @param clkdiv
 *   The clock divisor to use.
 *****************************************************************************/
void BOOTLDIO_usartInit( uint32_t clkdiv )
{
  /* Configure BOOTLOADER_USART */
  /* USART default to 1 stop bit, no parity, 8 data bits, so not
   * explicitly set */

#if defined( USART_OVERLAPS_WITH_BOOTLOADER )
  GPIO->ROUTE = 0;
#endif

  /* Set the clock division */
  BOOTLOADER_USART->CLKDIV = clkdiv;


  /* Enable RX and TX pins and set location 0 */
  BOOTLOADER_USART->ROUTE = BOOTLOADER_USART_LOCATION
                            | USART_ROUTE_RXPEN | USART_ROUTE_TXPEN;

  /* Clear RX/TX buffers, enable RX/TX */
  BOOTLOADER_USART->CMD = USART_CMD_CLEARRX | USART_CMD_CLEARTX
                          | USART_CMD_RXEN | USART_CMD_TXEN;
}
