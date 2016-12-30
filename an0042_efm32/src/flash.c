/**************************************************************************//**
 * @file flash.c
 * @brief Bootloader flash writing functions.
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
#include "flash.h"
#include "em_usb.h"

uint32_t flashSize;

/***************************************************************************//**
*
* @brief
*   Calculate flash size
*******************************************************************************/
void FLASH_CalcFlashSize(void)
{
  /* Get flash size from DEVINFO - Optimization of DEVINFO->MEMSIZE extraction.
   */
  flashSize = (*(uint16_t*)0x0FE081F8) * 1024;

  USB_PRINTF( "Flash size is %ld bytes.\r\n",
              flashSize );
}

/***************************************************************************//**
*
* @brief
*   Initializes the Flash programmer
*******************************************************************************/
void FLASH_init(void)
{
  /* Write MSC unlock code to enable interface */
  MSC->LOCK = MSC_UNLOCK_CODE;
  /* Enable memory controller */
  MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
  /* Enable DMA */
  DMA->CONFIG = DMA_CONFIG_EN;
  /* Setup the DMA control block. */
  DMA->CTRLBASE = 0x20000000;          // Use unused part of IRQ vector table !
}

/**************************************************************************//**
 *
 * Programs a single word into flash.
 *
 * @param adr is the address of the word to program.
 * @param data is the word to program.
 *
 * This function will program one word into the on-chip flash.
 * Programming consists of ANDing the new data with the existing data; in
 * other words bits that contain 1 can remain 1 or be changed to 0, but bits
 * that are 0 can not be changed to 1.  Therefore, a word can be programmed
 * multiple times so long as these rules are followed; if a program operation
 * attempts to change a 0 bit to a 1 bit, that bit will not have its value
 * changed.
 *
 * This function will not return until the data has been programmed.
 *****************************************************************************/
__ramfunc void FLASH_writeWord(uint32_t adr, uint32_t data)
{
  /* Load address */
  MSC->ADDRB    = adr;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Load data */
  MSC->WDATA = data;

  /* Trigger write once */
  MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;

  /* Waiting for the write to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}
}

/**************************************************************************//**
 *
 * Program flash.
 *
 * @param block_start is a pointer to the base of the flash.
 * @param offset_into_block is the offset to start writing.
 * @param count is the number of bytes to be programmed. Must be a multiple of
 * four.
 * @param buffer is a pointer to a buffer holding the data.
 *
 * This function uses DMA channel 0 to program a buffer of words into
 * onboard flash. It will start the DMA transfer, but will not wait for
 * it's completion. If a DMA transfer is alreay in progress when this
 * function is called, the function will stall waiting for the previous
 * transfer to complete.
 *
 * This function will program a sequence of words into the on-chip flash.
 * Programming consists of ANDing the new data with the existing data; in
 * other words bits that contain 1 can remain 1 or be changed to 0, but bits
 * that are 0 can not be changed to 1.  Therefore, a word can be programmed
 * multiple times so long as these rules are followed; if a program operation
 * attempts to change a 0 bit to a 1 bit, that bit will not have its value
 * changed.
 *
 * Since the flash is programmed a word at a time, the starting address and
 * byte count must both be multiples of four.  It is up to the caller to
 * verify the programmed contents, if such verification is required.
 *****************************************************************************/
__ramfunc void FLASH_writeBlock(void *block_start,
                                uint32_t offset_into_block,
                                uint32_t count,
                                uint8_t const *buffer)
{
  // Use unused part of IRQ vector table as DMA descriptor memory !
  #define pDescr ((DMA_DESCRIPTOR_TypeDef *)0x20000000)

  /* Wait for previous DMA transfer completion */
  while (DMA->CHENS & DMA_CHENS_CH0ENS){}
  /* Wait for last flash programming operation from previous transfer */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}

  /* Set up a basic memory to peripheral DMA transfer. */
  /* Load the start address into the MSC */
  MSC->ADDRB    = ((uint32_t) block_start) + offset_into_block;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Set the MSC as the destination. */
  pDescr->DSTEND = (void *)(&(MSC->WDATA));

  /* Set up the end pointer to copy from the buffer. */
  pDescr->SRCEND = (void *)(buffer + count - 4);

  /* Control information */
  pDescr->CTRL = DMA_CTRL_DST_INC_NONE       /* Do not increment destination */
                 | DMA_CTRL_DST_SIZE_WORD    /* Transfer whole words at a time */
                 | DMA_CTRL_SRC_INC_WORD     /* Write one word at the time */
                 | DMA_CTRL_SRC_SIZE_WORD    /* Transfer whole words at a time */
                 | DMA_CTRL_R_POWER_1
                 | DMA_CTRL_CYCLE_CTRL_BASIC /* Basic transfer */
                                             /* Number of transfers minus two. */
                                             /* This field contains the number of transfers minus 1. */
                                             /* Because one word is transerred using WRITETRIG we need to */
                                             /* Substract one more. */
                 | (((count / 4) - 2) << _DMA_CTRL_N_MINUS_1_SHIFT);

  /* Set channel to trigger on MSC ready for data */
  DMA->CH[0].CTRL = DMA_CH_CTRL_SOURCESEL_MSC | DMA_CH_CTRL_SIGSEL_MSCWDATA;

  /* Load first word into the DMA */
  MSC->WDATA = *((uint32_t *)(buffer));

  /* Activate channel 0 */
  DMA->CHENS = DMA_CHENS_CH0ENS;

  /* Trigger the transfer */
  MSC->WRITECMD = MSC_WRITECMD_WRITETRIG;

  #undef pDescr
}


/**************************************************************************//**
 *
 * Erase a block of flash.
 *
 * @param blockStart is the start address of the flash block to be erased.
 *
 * This function will erase one blocks on the on-chip flash.  After erasing,
 * the block will be filled with 0xff bytes.  Read-only and execute-only
 * blocks can not be erased.
 *
 * This function will not return until the block has been erased.
 *****************************************************************************/
__ramfunc void FLASH_eraseOneBlock(uint32_t blockStart)
{
  uint32_t acc = 0xFFFFFFFF;
  uint32_t *ptr;

  /* Optimization - check if block is allready erased.
   * This will typically happen when the chip is new. */
  for (ptr = (uint32_t *) blockStart; ptr < (uint32_t *)(blockStart + FLASH_PAGE_SIZE); ptr++)
    acc &= *ptr;

  /* If the accumulator is unchanged, there is no need to do an erase. */
  if (acc == 0xFFFFFFFF)
    return;

  /* Load address */
  MSC->ADDRB    = blockStart;
  MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

  /* Send Erase Page command */
  MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

  /* Waiting for erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}
}


/**************************************************************************//**
 *
 * Mass erase a flash block.
 *
 * @param eraseCmd The mass erase command to write to MSC WRITECMD register.
 *
 * This function will mass erase a 512K block on a Giant device.
 * This function will not return until the block has been erased.
 *****************************************************************************/
__ramfunc void FLASH_massErase( uint32_t eraseCmd )
{
  /* Unlock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_UNLOCK;

  /* Send Mass erase command */
  MSC->WRITECMD = eraseCmd;

  /* Waiting for erase to complete */
  while ((MSC->STATUS & MSC_STATUS_BUSY)){}

  /* Lock mass erase */
  MSC->MASSLOCK = MSC_MASSLOCK_LOCKKEY_LOCK;
}
