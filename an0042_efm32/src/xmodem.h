/**************************************************************************//**
 * @file xmodem.h
 * @brief XMODEM prototypes and definitions
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
#ifndef _XMODEM_H
#define _XMODEM_H

#if defined( NO_RAMFUNCS )
#define __ramfunc
#endif

#define XMODEM_SOH                1
#define XMODEM_EOT                4
#define XMODEM_ACK                6
#define XMODEM_NAK                21
#define XMODEM_CAN                24
#define XMODEM_NCG                67

#define XMODEM_DATA_SIZE          128

typedef struct
{
  uint8_t padding; /* Padding to make sure data is 32 bit aligned. */
  uint8_t header;
  uint8_t packetNumber;
  uint8_t packetNumberC;
  uint8_t data[XMODEM_DATA_SIZE];
  uint8_t crcHigh;
  uint8_t crcLow;
} XMODEM_packet;

__ramfunc int XMODEM_download(uint32_t baseAddress, uint32_t endAddress);

#endif
