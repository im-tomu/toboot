/*
 * Fadecandy DFU Bootloader
 * 
 * Copyright (c) 2013 Micah Elizabeth Scott
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdbool.h>
#include "mcu.h"
#include "usb_dev.h"
#include "dfu.h"

// Internal flash-programming state machine
static unsigned fl_current_addr = 0;
static enum {
    flsIDLE = 0,
    flsERASING,
    flsPROGRAMMING
} fl_state;

static dfu_state_t dfu_state = dfuIDLE;
static dfu_status_t dfu_status = OK;
static unsigned dfu_poll_timeout = 1;

static uint32_t dfu_buffer[DFU_TRANSFER_SIZE/4];
static uint32_t dfu_buffer_offset;
static uint32_t fl_num_words;

// Memory offset we're uploading to.
static uint32_t dfu_target_address;

bool fl_is_idle(void) {
    return fl_state == flsIDLE;
}

void *memcpy(void *dst, const void *src, size_t cnt) {
    uint8_t *dst8 = dst;
    const uint8_t *src8 = src;
    while (cnt > 0) {
        cnt--;
        *(dst8++) = *(src8++);
    }
    return dst;
}

static bool ftfl_busy()
{
    // Is the flash memory controller busy?
    return (MSC->STATUS & MSC_STATUS_BUSY);
}

static void ftfl_busy_wait()
{
    // Wait for the flash memory controller to finish any pending operation.
    while (ftfl_busy());
}

static void ftfl_begin_erase_sector(uint32_t address)
{
    // Erase the page at the specified address.
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    ftfl_busy_wait();
    MSC->ADDRB = address;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;
}

static void ftfl_begin_program_section(uint32_t address)
{
//    if (address == 0x3c00)
//        asm("bkpt #23");
    // Write the buffer word to the currently selected address.
    // Note that after this is done, the address is incremented by 4.
    dfu_buffer_offset = 0;
    dfu_target_address = address;
    fl_num_words--;
ftfl_busy_wait();
    MSC->ADDRB = address;
    if ((dfu_target_address & 0x3ff))
        asm("bkpt #232");
ftfl_busy_wait();
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
    MSC->WDATA = dfu_buffer[dfu_buffer_offset++];
    MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;
}

static uint32_t address_for_block(unsigned blockNum)
{
    return 0x2000 + (blockNum << 10);
}

void dfu_init(void)
{
    CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY))
        ;

    /* Unlock the MSC */
    MSC->LOCK = MSC_UNLOCK_CODE;

    // Enable writing to flash
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
    MSC->IEN |= MSC_IEN_WRITE | MSC_IEN_ERASE;
    NVIC_EnableIRQ(MSC_IRQn);
}

uint8_t dfu_getstate(void)
{
    return dfu_state;
}

bool dfu_download(unsigned blockNum, unsigned blockLength,
    unsigned packetOffset, unsigned packetLength, const uint8_t *data)
{
    if (packetOffset + packetLength > DFU_TRANSFER_SIZE ||
        packetOffset + packetLength > blockLength) {

        // Overflow!
        dfu_state = dfuERROR;
        dfu_status = errADDRESS;
        return false;
    }

    // Store more data...
    memcpy(((uint8_t *)dfu_buffer) + packetOffset, data, packetLength);

    if (packetOffset + packetLength != blockLength) {
        // Still waiting for more data.
        return true;
    }

    if (dfu_state != dfuIDLE && dfu_state != dfuDNLOAD_IDLE) {
        // Wrong state! Oops.
        dfu_state = dfuERROR;
        dfu_status = errSTALLEDPKT;
        return false;
    }

    if (ftfl_busy() || fl_state != flsIDLE) {
        // Flash controller shouldn't be busy now!
        dfu_state = dfuERROR;
        dfu_status = errUNKNOWN;
        return false;       
    }

    if (!blockLength) {
        // End of download
//        asm("bkpt #0");
        dfu_state = dfuMANIFEST_SYNC;
        dfu_status = OK;
        return true;
    }

    // Start programming a block by erasing the corresponding flash sector
    fl_state = flsERASING;
    fl_current_addr = address_for_block(blockNum);
    fl_num_words = blockLength / 4;
//    if (fl_current_addr == 0x3c00)
//        asm("bkpt #93");
    ftfl_begin_erase_sector(fl_current_addr);

    dfu_state = dfuDNLOAD_SYNC;
    dfu_status = OK;
    return true;
}

static bool fl_handle_status(uint8_t fstat, unsigned specificError)
{
    /*
     * Handle common errors from an FSTAT register value.
     * The indicated "specificError" is used for reporting a command-specific
     * error from MGSTAT0.
     *
     * Returns true if handled, false if not.
     */

    if (fstat & MSC_STATUS_BUSY) {
        // Still working...
        return true;
    }

    if (fstat & (MSC_STATUS_ERASEABORTED | MSC_STATUS_WORDTIMEOUT)) {
        // Bus collision. We did something wrong internally.
        dfu_state = dfuERROR;
        dfu_status = errUNKNOWN;
        fl_state = flsIDLE;
        return true;
    }

    if (fstat & (MSC_STATUS_INVADDR | MSC_STATUS_LOCKED)) {
        // Address or protection error
        dfu_state = dfuERROR;
        dfu_status = errADDRESS;
        fl_state = flsIDLE;
        return true;
    }

    if (fl_state == flsPROGRAMMING) {
        // Still programming...
        return true;
    }

    return false;
}

static void fl_state_poll(void)
{
    // Try to advance the state of our own flash programming state machine.

    uint32_t fstat = MSC->STATUS;

    switch (fl_state) {

        case flsIDLE:
            break;

        case flsERASING:
            if (!fl_handle_status(fstat, errERASE)) {
                // Done! Move on to programming the sector.
                fl_state = flsPROGRAMMING;
                ftfl_begin_program_section(fl_current_addr);
            }
            break;

        case flsPROGRAMMING:
            if (!fl_handle_status(fstat, errVERIFY)) {
                // Done!
                fl_state = flsIDLE;
            }
            break;
    }
}

bool dfu_getstatus(uint8_t status[8])
{
    switch (dfu_state) {

        case dfuDNLOAD_SYNC:
        case dfuDNBUSY:
            // Programming operation in progress. Advance our private flash state machine.
            fl_state_poll();

            if (dfu_state == dfuERROR) {
                // An error occurred inside fl_state_poll();
            } else if (fl_state == flsIDLE) {
                dfu_state = dfuDNLOAD_IDLE;
            } else {
                dfu_state = dfuDNBUSY;
            }
            break;

        case dfuMANIFEST_SYNC:
            // Ready to reboot. The main thread will take care of this. Also let the DFU tool
            // know to leave us alone until this happens.
            dfu_state = dfuMANIFEST;
            dfu_poll_timeout = 1000;
            break;

        default:
            break;
    }

    status[0] = dfu_status;
    status[1] = dfu_poll_timeout;
    status[2] = dfu_poll_timeout >> 8;
    status[3] = dfu_poll_timeout >> 16;
    status[4] = dfu_state;
    status[5] = 0;  // iString

    return true;
}

bool dfu_clrstatus(void)
{
    switch (dfu_state) {

    case dfuERROR:
        // Clear an error
        dfu_state = dfuIDLE;
        dfu_status = OK;
        return true;

    default:
        // Unexpected request
        dfu_state = dfuERROR;
        dfu_status = errSTALLEDPKT;
        return false;
    }
}

bool dfu_abort(void)
{
    dfu_state = dfuIDLE;
    dfu_status = OK;
    return true;
}

void MSC_Handler(void) {
    uint32_t msc_irq_reason = MSC->IF;

    // ERASE interrupt will happen once an erase has completed,
    // and we need to start writing words.
    if (msc_irq_reason & MSC_IF_ERASE) {
        // Set target address to the desired target address,
        // since the address was cleared after the erase finished.
        //MSC->ADDRB = dfu_target_address;
        //MSC->WRITECMD = MSC_WRITECMD_LADDRIM;

        // Write the buffer word to the currently selected address.
        // Note that after this is done, the address is incremented by 4.
    }

    if (msc_irq_reason & MSC_IF_WRITE) {
        // Write the buffer word to the currently selected address.
        // Note that after this is done, the address is incremented by 4.
        if (fl_num_words > 0) {
            fl_num_words--;
            dfu_target_address += 4;
ftfl_busy_wait();
            if (!(dfu_target_address & 0x3ff))
                asm("bkpt #231");
            MSC->ADDRB = dfu_target_address;
ftfl_busy_wait();
            MSC->WDATA = dfu_buffer[dfu_buffer_offset++]; 
            MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;
        }
        else {
            // Move to the IDLE state only if we're out of data to write.
            fl_state = flsIDLE;
        }
    }

    // Clear iterrupts so we don't fire again.
    MSC->IFC = MSC_IFC_ERASE | MSC_IFC_WRITE;
}