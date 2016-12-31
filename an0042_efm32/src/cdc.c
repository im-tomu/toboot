/**************************************************************************//**
 * @file cdc.c
 * @brief CDC source file
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
#include "em_usb.h"
#include "cdc.h"

volatile bool CDC_Configured = false;

/* The serial port LINE CODING data structure, used to carry information  */
/* about serial port baudrate, parity etc. between host and device.       */
EFM32_PACK_START( 1 )
typedef struct
{
  uint32_t    dwDTERate;          /** Baudrate                            */
  uint8_t     bCharFormat;        /** Stop bits, 0=1 1=1.5 2=2            */
  uint8_t     bParityType;        /** 0=None 1=Odd 2=Even 3=Mark 4=Space  */
  uint8_t     bDataBits;          /** 5, 6, 7, 8 or 16                    */
  uint8_t     dummy;              /** To ensure size is a multiple of 4 bytes.*/
} __attribute__ ((packed)) cdcLineCoding_TypeDef;
EFM32_PACK_END()

/*
 * The LineCoding variable must be 4-byte aligned as it is used as USB
 * transmit and receive buffer
 */
EFM32_ALIGN(4)
EFM32_PACK_START( 1 )
cdcLineCoding_TypeDef __attribute__ ((aligned(4))) cdcLineCoding =
{
  115200, 0, 0, 8, 0
};
EFM32_PACK_END()

/**************************************************************************//**
 * Called each time the USB device state is changed.
 * Starts CDC operation when device has been configured by USB host.
 *****************************************************************************/
__ramfunc void CDC_StateChange( USBD_State_TypeDef oldState, USBD_State_TypeDef newState )
{
  if ( newState == USBD_STATE_CONFIGURED )
  {
    CDC_Configured = true;
  }
  else
  {
    CDC_Configured = false;
  }
}


/**************************************************************************//**
 * Called each time the USB host sends a SETUP command.
 * Implements CDC class specific commands.
 *****************************************************************************/
__ramfunc int CDC_SetupCmd( const USB_Setup_TypeDef *setup )
{
  int retVal = USB_STATUS_REQ_UNHANDLED;

  if ( ( setup->Type      == USB_SETUP_TYPE_CLASS          ) &&
       ( setup->Recipient == USB_SETUP_RECIPIENT_INTERFACE )    )
  {
    switch ( setup->bRequest )
    {
      case USB_CDC_GETLINECODING:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wValue    == 0 ) &&
             ( setup->wIndex    == 0 ) &&       /* Interface no.            */
             ( setup->wLength   == 7 ) &&       /* Length of cdcLineCoding  */
             ( setup->Direction == USB_SETUP_DIR_IN ) )
#endif
        {
          /* Send current settings to USB host. */
          USBD_Write( 0, (void*)&cdcLineCoding, 7, NULL );
          retVal = USB_STATUS_OK;
        }
        break;

      case USB_CDC_SETLINECODING:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wValue    == 0 ) &&
             ( setup->wIndex    == 0 ) &&       /* Interface no.            */
             ( setup->wLength   == 7 ) &&       /* Length of cdcLineCoding  */
             ( setup->Direction != USB_SETUP_DIR_IN ) )
#endif
        {
          /* Get new settings from USB host. */
          USBD_Read( 0, (void*)&cdcLineCoding, 7, NULL );
          retVal = USB_STATUS_OK;
        }
        break;

      case USB_CDC_SETCTRLLINESTATE:
      /********************/
#if defined( NO_RAMFUNCS )
        if ( ( setup->wIndex    == 0 ) &&       /* Interface no.  */
             ( setup->wLength   == 0 ) )        /* No data        */
#endif
        {
          /* Do nothing ( Non compliant behaviour !! ) */
          retVal = USB_STATUS_OK;
        }
        break;
    }
  }

  return retVal;
}
