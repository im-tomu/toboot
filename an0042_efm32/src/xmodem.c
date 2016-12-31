/**************************************************************************//**
 * @file xmodem.c
 * @brief XMODEM protocol
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
#include "em_common.h"
#include "xmodem.h"
#include "bootldio.h"
#include "flash.h"
#include "crc.h"
#include "config.h"
#include "em_usb.h"

#define ALIGNMENT(base,align) (((base)+((align)-1))&(~((align)-1)))

/* Packet storage. Double buffered version. */
#pragma data_alignment=4
uint8_t rawPacket[2][ALIGNMENT(sizeof(XMODEM_packet),4)];

/**************************************************************************//**
 * @brief Verifies checksum, packet numbering and
 * @param pkt The packet to verify
 * @param sequenceNumber The current sequence number.
 * @returns -1 on packet error, 0 otherwise
 *****************************************************************************/
__ramfunc static int VerifyPacketChecksum(XMODEM_packet *pkt, int sequenceNumber)
{
  uint16_t packetCRC;
  uint16_t calculatedCRC;

  /* Check the packet number integrity */
  if (pkt->packetNumber + pkt->packetNumberC != 255)
  {
    return -1;
  }

  /* Check that the packet number matches the excpected number */
  if (pkt->packetNumber != (sequenceNumber % 256))
  {
    return -1;
  }

  calculatedCRC = CRC_calc((uint8_t *) pkt->data, (uint8_t *) &(pkt->crcHigh));
  packetCRC     = pkt->crcHigh << 8 | pkt->crcLow;

  /* Check the CRC value */
  if (calculatedCRC != packetCRC)
  {
    return -1;
  }
  return 0;
}

/**************************************************************************//**
 * @brief Starts a XMODEM download.
 *
 * @param baseAddress
 *   The address to start writing from
 *
 * @param endAddress
 *   The last address. This is only used for clearing the flash
 *****************************************************************************/
__ramfunc int XMODEM_download(uint32_t baseAddress, uint32_t endAddress)
{
  XMODEM_packet *pkt;
  uint32_t      i;
  uint32_t      addr;
  uint32_t      sequenceNumber = 1;

  /* Erase flash */
  addr = baseAddress;

  /* Do userpage and lockpage first. */
  if ( ( addr == USERDATA_BASE ) || ( addr == LOCKBITS_BASE ) )
  {
    FLASH_eraseOneBlock( addr );
  }
  else
  {
    /* Do first mass-erase block. */
    /* Check if it is possible to mass erase first block. */
    if ( ( addr == 0 ) && ( endAddress >= MASSERASE_BLOCK_SIZE ) )
    {
      FLASH_massErase( MSC_WRITECMD_ERASEMAIN0 );
      addr += MASSERASE_BLOCK_SIZE;
    }
    else
    {
      while ( addr < MASSERASE_BLOCK_SIZE )
      {
        FLASH_eraseOneBlock( addr );
        addr += FLASH_PAGE_SIZE;
      }
    }

#if defined( MSC_WRITECMD_ERASEMAIN1 )
    /* Do next mass-erase block (if any at all). */
    if ( flashSize > MASSERASE_BLOCK_SIZE )
    {
      /* Mass erase possible ? */
      if ( ( addr        == MASSERASE_BLOCK_SIZE      ) &&
           ( endAddress  >= 2 * MASSERASE_BLOCK_SIZE  )    )
      {
        FLASH_massErase( MSC_WRITECMD_ERASEMAIN1 );
      }
      else
      {
        while ( addr < 2 * MASSERASE_BLOCK_SIZE )
        {
          FLASH_eraseOneBlock( addr );
          addr += FLASH_PAGE_SIZE;
        }
      }
    }
#endif
  }

  /*
   * Send one start transmission packet. Wait for a response. If there is no
   * response, we resend the start transmission packet.
   * Note: There is a fairly long delay between retransmissions (~6 s).
   */

  pkt = (XMODEM_packet*)rawPacket[ sequenceNumber ];

  while (1)
  {
    BOOTLDIO_txByte(XMODEM_NCG);

    if ( BOOTLDIO_usbMode() )
    {
      if ( BOOTLDIO_getPacket( pkt, 6000 ) )
      {
        goto usb_loop_entry;
      }
    }
    else
    {
      for (i = 0; i < 20000000; i++)
      {
#if defined( BOOTLOADER_LEUART_CLOCKEN )
        if (BOOTLOADER_USART->STATUS & LEUART_STATUS_RXDATAV)
#else
        if (BOOTLOADER_USART->STATUS & USART_STATUS_RXDATAV)
#endif
        {
          goto xmodem_transfer;
        }
      }
    }
  }

xmodem_transfer:
  while (1)
  {
    /* Swap buffer for packet buffer */
    pkt = (XMODEM_packet*)rawPacket[ sequenceNumber & 1 ];

    if ( BOOTLDIO_usbMode() )
    {
      BOOTLDIO_getPacket( pkt, 0 );
    }
    else
    {
      /* Fetch the first byte of the packet explicitly, as it defines the
       * rest of the packet */
      pkt->header = BOOTLDIO_rxByte();
    }

usb_loop_entry:
    /* Check for end of transfer */
    if (pkt->header == XMODEM_EOT)
    {
      /* Acknowledge End of transfer */
      BOOTLDIO_txByte(XMODEM_ACK);
      break;
    }

    /* If the header is not a start of header (SOH), then cancel
     * the transfer. */
    if (pkt->header != XMODEM_SOH)
    {
      return -1;
    }

    if ( !BOOTLDIO_usbMode() )
    {
      /* Fill the remaining bytes packet */
      /* Byte 0 is padding, byte 1 is header */
      for ( i = 2; i < sizeof(XMODEM_packet); i++ )
      {
        *(((uint8_t *) pkt) + i) = BOOTLDIO_rxByte();
      }
    }

    if ( VerifyPacketChecksum( pkt, sequenceNumber ) != 0 )
    {
      /* On a malformed packet, we send a NAK, and start over */
      BOOTLDIO_txByte(XMODEM_NAK);
      continue;
    }

    /* Write data to flash */
    FLASH_writeBlock((void *) baseAddress,
                     (sequenceNumber - 1) * XMODEM_DATA_SIZE,
                     XMODEM_DATA_SIZE,
                     (uint8_t const *) pkt->data);



    sequenceNumber++;
    /* Send ACK */
    BOOTLDIO_txByte(XMODEM_ACK);
  }

  /* Wait for previous DMA transfer completion */
  while (DMA->CHENS & DMA_CHENS_CH0ENS){}
  /* Wait for last flash programming operation from previous transfer */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}

  return 0;
}
