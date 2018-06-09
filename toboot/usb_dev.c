/*
 * Simple control-only USB driver for DFU bootloader mode.
 * Originally based on Chopstx USB driver:
 * 
 * Chopstx USB Driver
 * Copyright (C) 2017 Sergei Glushchenko
 * Author: Sergei Glushchenko <gl.sergei@gmail.com>
 *
 * Portions taken from Teensyduino.
 * Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As additional permission under GNU GPL version 3 section 7, you may
 * distribute non-source form of the Program without the copy of the
 * GNU GPL normally required by section 4, provided you inform the
 * recipients of GNU GPL by a written offer.
 *
 */

#include "mcu.h"
#include "usb_dev.h"
#include "usb_desc.h"
#include "dfu.h"

#define STANDARD_ENDPOINT_DESC_SIZE 0x09
#define USB_MAX_PACKET_SIZE 64 /* For FS device */

static uint8_t usb_configuration = 0;

void *memcpy(void *dst, const void *src, size_t cnt);
static uint32_t ep0_rx_offset;

static struct device_req ep0_setup_pkt[3] __attribute__((aligned(4)));
static char ctrl_send_buf[USB_MAX_PACKET_SIZE] __attribute__((aligned(4)));
static uint8_t rx_buffer[USB_MAX_PACKET_SIZE];

/* The state machine states of a control pipe */
enum CONTROL_STATE
{
    WAIT_SETUP,
    IN_DATA,
    OUT_DATA,
    LAST_IN_DATA,
    WAIT_STATUS_IN,
    WAIT_STATUS_OUT,
    STALLED,
};

struct ctrl_data
{
    uint8_t *addr;
    uint16_t len;
    uint8_t require_zlp;
};

struct usb_dev
{
    struct device_req dev_req;
    struct ctrl_data ctrl_data;
    enum CONTROL_STATE state;
};

static struct usb_dev default_dev;
static struct device_req last_setup;
struct usb_dev *dev = &default_dev;
static uint8_t reply_buffer[8];

static void efm32hg_connect(void)
{
    USB->DCTL &= ~(DCTL_WO_BITMASK | USB_DCTL_SFTDISCON);
}

static void efm32hg_set_daddr(uint8_t daddr)
{
    USB->DCFG = (USB->DCFG & ~_USB_DCFG_DEVADDR_MASK) | (daddr << 4);
}

static void efm32hg_prepare_ep0_setup(void)
{
    USB->DOEP0TSIZ = (8 * 3 << 0) /* XFERSIZE */
                     | (1 << 19)  /* PKTCNT */
                     | (3 << 29); /* SUPCNT */
    USB->DOEP0DMAADDR = (uint32_t)&ep0_setup_pkt;
    USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DOEP0CTL_EPENA;
}

static void efm32hg_prepare_ep0_out(const void *buf, size_t len)
{
    USB->DOEP0DMAADDR = (uint32_t)buf;
    USB->DOEP0TSIZ = (len << 0)   /* XFERSIZE */
                     | (1 << 19); /* PKTCNT */
    USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DOEP0CTL_CNAK | USB_DOEP0CTL_EPENA;
}

static void efm32hg_prepare_ep0_in(const void *buf, size_t len)
{
    USB->DIEP0DMAADDR = (uint32_t)buf;
    USB->DIEP0TSIZ = (len << 0)   /* XFERSIZE */
                     | (1 << 19); /* PKTCNT */
    USB->DIEP0CTL = (USB->DIEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DIEP0CTL_CNAK | USB_DIEP0CTL_EPENA;
}

static void efm32hg_ep0_out_stall(void)
{
    USB_DOUTEPS[0].CTL = (USB_DOUTEPS[0].CTL & ~DEPCTL_WO_BITMASK) | USB_DIEP_CTL_STALL;
}

static void
efm32hg_ep0_in_stall(void)
{
    uint32_t ctl = USB_DOUTEPS[0].CTL & ~DEPCTL_WO_BITMASK;

    USB_DINEPS[0].CTL |= USB_DIEP_CTL_STALL;
    if (ctl & USB_DIEP_CTL_EPENA)
        ctl |= USB_DIEP_CTL_EPDIS;
    USB_DINEPS[0].CTL = ctl;
}

static void handle_datastage_out(struct usb_dev *dev)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len = USB->DOEP0TSIZ & 0x7FUL; /* XFERSIZE */
    uint32_t pktsize = 64;

    data_p->len -= len;
    data_p->addr += len;

    if (data_p->len == 0)
    {
        /* No more data to receive, proceed to send acknowledge for IN.  */
        efm32hg_prepare_ep0_setup();
        dev->state = WAIT_STATUS_IN;
        efm32hg_prepare_ep0_in(NULL, 0);
    }
    else
    {
        len = data_p->len < pktsize ? data_p->len : pktsize;
        dev->state = OUT_DATA;
        efm32hg_prepare_ep0_out(data_p->addr, len);
    }
}

static void handle_datastage_in(struct usb_dev *dev)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len = 64;

    if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
        if (data_p->require_zlp)
        {
            data_p->require_zlp = 0;

            efm32hg_prepare_ep0_setup();
            /* No more data to send.  Send empty packet */
            efm32hg_prepare_ep0_in(NULL, 0);
        }
        else
        {
            /* No more data to send, proceed to receive OUT acknowledge.  */
            dev->state = WAIT_STATUS_OUT;
            efm32hg_prepare_ep0_out(NULL, 0);
        }

        return;
    }

    dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

    if (len > data_p->len)
        len = data_p->len;

    efm32hg_prepare_ep0_setup();
    efm32hg_prepare_ep0_in(data_p->addr, len);
    data_p->len -= len;
    data_p->addr += len;
}

void usb_lld_ctrl_recv(struct usb_dev *dev, void *p, size_t len)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t pktsize = 64;
    data_p->addr = (uint8_t *)p;
    data_p->len = len;
    if (len > pktsize)
        len = pktsize;

    efm32hg_prepare_ep0_out(p, len);
    dev->state = OUT_DATA;
}

static void usb_lld_ctrl_ack(struct usb_dev *dev)
{
    /* Zero length packet for ACK.  */
    efm32hg_prepare_ep0_setup();
    dev->state = WAIT_STATUS_IN;
    efm32hg_prepare_ep0_in(NULL, 0);
}

/*
 * BUF: Pointer to data memory.  Data memory should not be allocated
 *      on stack when BUFLEN > USB_MAX_PACKET_SIZE.
 *
 * BUFLEN: size of the data.
 */
void usb_lld_ctrl_send(struct usb_dev *dev, const void *buf, size_t buflen)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len_asked = dev->dev_req.wLength;
    uint32_t len;
    uint32_t pktsize = 64;

    data_p->addr = (void *)buf;
    data_p->len = buflen;

    /* Restrict the data length to be the one host asks for */
    if (data_p->len > len_asked)
        data_p->len = len_asked;

    data_p->require_zlp = (data_p->len != 0 && (data_p->len & (pktsize - 1)) == 0);

    if (((uint32_t)data_p->addr & 3) && (data_p->len <= pktsize))
    {
        data_p->addr = (void *)ctrl_send_buf;
        memcpy(data_p->addr, buf, buflen);
    }

    if (data_p->len < pktsize)
    {
        len = data_p->len;
        dev->state = LAST_IN_DATA;
    }
    else
    {
        len = pktsize;
        dev->state = IN_DATA;
    }

    efm32hg_prepare_ep0_in(data_p->addr, len);

    data_p->len -= len;
    data_p->addr += len;
}

static void usb_lld_ctrl_error(struct usb_dev *dev)
{
    efm32hg_ep0_out_stall();
    efm32hg_ep0_in_stall();
    dev->state = WAIT_SETUP;
    efm32hg_prepare_ep0_setup();
}

static void handle_out0(struct usb_dev *dev)
{
    if (dev->state == OUT_DATA)
    {
        /* It's normal control WRITE transfer.  */
        handle_datastage_out(dev);

        // The only control OUT request we have now, DFU_DNLOAD
        if (last_setup.wRequestAndType == 0x0121)
        {
            if (last_setup.wIndex != 0 && ep0_rx_offset > last_setup.wLength)
            {
                usb_lld_ctrl_error(dev);
            }
            else
            {
                uint32_t size = last_setup.wLength - ep0_rx_offset;
                if (size > EP0_SIZE)
                    size = EP0_SIZE;

                if (dfu_download(last_setup.wValue,  // blockNum
                                 last_setup.wLength, // blockLength
                                 ep0_rx_offset,      // packetOffset
                                 size,               // packetLength
                                 rx_buffer))
                {
                    ep0_rx_offset += size;
                    if (ep0_rx_offset >= last_setup.wLength)
                    {
                        // End of transaction, acknowledge with a zero-length IN
                        usb_lld_ctrl_ack(dev);
                    }
                }
                else
                {
                    usb_lld_ctrl_error(dev);
                }
            }
        }
    }
    else if (dev->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
        efm32hg_prepare_ep0_setup();
        ep0_rx_offset = 0;
        dev->state = WAIT_SETUP;
    }
    else
    {
        /*
       * dev->state == IN_DATA || dev->state == LAST_IN_DATA
       * (Host aborts the transfer before finish)
       * Or else, unexpected state.
       * STALL the endpoint, until we receive the next SETUP token.
       */
        dev->state = STALLED;
        efm32hg_ep0_out_stall();
        efm32hg_ep0_in_stall();
        dev->state = WAIT_SETUP;
        efm32hg_prepare_ep0_setup();
    }
}

static void usb_setup(struct usb_dev *dev)
{
    const uint8_t *data = NULL;
    uint32_t datalen = 0;
    const usb_descriptor_list_t *list;
    last_setup = dev->dev_req;

    switch (dev->dev_req.wRequestAndType)
    {
    case 0x0500: // SET_ADDRESS
        efm32hg_set_daddr(dev->dev_req.wValue);
        break;
    case 0x0900: // SET_CONFIGURATION
        usb_configuration = dev->dev_req.wValue;
        break;
    case 0x0880: // GET_CONFIGURATION
        reply_buffer[0] = usb_configuration;
        datalen = 1;
        data = reply_buffer;
        break;
    case 0x0080: // GET_STATUS (device)
        reply_buffer[0] = 0;
        reply_buffer[1] = 0;
        datalen = 2;
        data = reply_buffer;
        break;
    case 0x0082: // GET_STATUS (endpoint)
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        reply_buffer[0] = 0;
        reply_buffer[1] = 0;

        if (USB->DIEP0CTL & USB_DIEP_CTL_STALL)
            reply_buffer[0] = 1;
        data = reply_buffer;
        datalen = 2;
        break;
    case 0x0102: // CLEAR_FEATURE (endpoint)
        if (dev->dev_req.wIndex > 0 || dev->dev_req.wValue != 0)
        {
            // TODO: do we need to handle IN vs OUT here?
            usb_lld_ctrl_error(dev);
            return;
        }
        USB->DIEP0CTL &= ~USB_DIEP_CTL_STALL;
        // TODO: do we need to clear the data toggle here?
        break;
    case 0x0302: // SET_FEATURE (endpoint)
        if (dev->dev_req.wIndex > 0 || dev->dev_req.wValue != 0)
        {
            // TODO: do we need to handle IN vs OUT here?
            usb_lld_ctrl_error(dev);
            return;
        }
        USB->DIEP0CTL |= USB_DIEP_CTL_STALL;
        // TODO: do we need to clear the data toggle here?
        break;
    case 0x0680: // GET_DESCRIPTOR
    case 0x0681:
        for (list = usb_descriptor_list; 1; list++)
        {
            if (list->addr == NULL)
                break;
            if (dev->dev_req.wValue == list->wValue)
            {
                data = list->addr;
                if ((dev->dev_req.wValue >> 8) == 3)
                {
                    // for string descriptors, use the descriptor's
                    // length field, allowing runtime configured
                    // length.
                    datalen = *(list->addr);
                }
                else
                {
                    datalen = list->length;
                }
                goto send;
            }
        }
        usb_lld_ctrl_error(dev);
        return;

    case (MSFT_VENDOR_CODE << 8) | 0xC0: // Get Microsoft descriptor
    case (MSFT_VENDOR_CODE << 8) | 0xC1:
        if (dev->dev_req.wIndex == 0x0004)
        {
            // Return WCID descriptor
            data = usb_microsoft_wcid;
            datalen = MSFT_WCID_LEN;
            break;
        }
        usb_lld_ctrl_error(dev);
        return;

    case (WEBUSB_VENDOR_CODE << 8) | 0xC0: // Get WebUSB descriptor
        if (dev->dev_req.wIndex == 0x0002)
        {
            if (dev->dev_req.wValue == 0x0001)
            {
                // Return landing page URL descriptor
                data = (uint8_t*)&landing_url_descriptor;
                datalen = LANDING_PAGE_DESCRIPTOR_SIZE;
                break;
            }
        }
        usb_lld_ctrl_error(dev);
        return;

    case 0x0121: // DFU_DNLOAD
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        // Data comes in the OUT phase. But if it's a zero-length request, handle it now.
        if (dev->dev_req.wLength == 0)
        {
            if (!dfu_download(dev->dev_req.wValue, 0, 0, 0, NULL))
            {
                usb_lld_ctrl_error(dev);
                return;
            }
            usb_lld_ctrl_ack(dev);
            return;
        }
        unsigned int len = dev->dev_req.wLength;
        if (len > sizeof(rx_buffer))
            len = sizeof(rx_buffer);
        usb_lld_ctrl_recv(dev, rx_buffer, len);
        return;

    case 0x03a1: // DFU_GETSTATUS
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        if (dfu_getstatus(reply_buffer))
        {
            data = reply_buffer;
            datalen = 6;
            break;
        }
        else
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        break;

    case 0x0421: // DFU_CLRSTATUS
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        if (dfu_clrstatus())
        {
            break;
        }
        else
        {
            usb_lld_ctrl_error(dev);
            return;
        }

    case 0x05a1: // DFU_GETSTATE
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        reply_buffer[0] = dfu_getstate();
        data = reply_buffer;
        datalen = 1;
        break;

    case 0x0621: // DFU_ABORT
        if (dev->dev_req.wIndex > 0)
        {
            usb_lld_ctrl_error(dev);
            return;
        }
        if (dfu_abort())
        {
            break;
        }
        else
        {
            usb_lld_ctrl_error(dev);
            return;
        }

    default:
        usb_lld_ctrl_error(dev);
        return;
    }

send:
    if (data && datalen)
        usb_lld_ctrl_send(dev, data, datalen);
    else
        usb_lld_ctrl_ack(dev);
    return;
}

static void handle_in0(struct usb_dev *dev)
{
    if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
    {
        handle_datastage_in(dev);
    }
    else if (dev->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
        efm32hg_prepare_ep0_setup();
        dev->state = WAIT_SETUP;
    }
    else
    {
        dev->state = STALLED;
        efm32hg_ep0_out_stall();
        efm32hg_ep0_in_stall();
        dev->state = WAIT_SETUP;
        efm32hg_prepare_ep0_setup();
    }
}

__attribute__ ((section(".startup")))
void USB_Handler(void)
{
    uint32_t intsts = USB->GINTSTS & USB->GINTMSK;

    if (intsts & USB_GINTSTS_USBRST)
    {
        USB->GINTSTS = USB_GINTSTS_USBRST;
        efm32hg_set_daddr(0);
        return;
    }

    if (intsts & USB_GINTSTS_ENUMDONE)
    {
        USB->GINTSTS = USB_GINTSTS_ENUMDONE;
        efm32hg_prepare_ep0_setup();
        dev->state = WAIT_SETUP;
    }

    if (intsts & USB_GINTSTS_IEPINT)
    {
        uint32_t sts = USB->DIEP0INT & USB->DIEPMSK;
        if (sts & USB_DIEP_INT_XFERCOMPL)
        {
            USB->DIEP0INT = USB_DIEP_INT_XFERCOMPL;
            handle_in0(dev);
        }
    }

    if (intsts & USB_GINTSTS_OEPINT)
    {
        uint32_t sts = USB->DOEP0INT & USB->DOEPMSK;

        if (sts & USB_DOEP0INT_STUPPKTRCVD)
        {
            USB->DOEP0INT = USB_DOEP0INT_STUPPKTRCVD;
            sts &= ~USB_DOEP_INT_XFERCOMPL;
        }

        if (sts & USB_DOEP_INT_XFERCOMPL)
        {
            USB->DOEP0INT = USB_DOEP_INT_XFERCOMPL;

            USB->DOEP0INT = USB_DOEP0INT_STUPPKTRCVD;

            if (sts & USB_DOEP0INT_SETUP)
            {
                USB->DOEP0INT = USB_DOEP0INT_SETUP;
                int supcnt = (USB->DOEP0TSIZ & 0x60000000UL) >> 29;
                supcnt = (supcnt == 3) ? 2 : supcnt;
                dev->dev_req = ep0_setup_pkt[2 - supcnt];
                usb_setup(dev);
            }
            else if (dev->state == WAIT_STATUS_IN)
                dev->state = WAIT_SETUP;
            else if (dev->state != WAIT_SETUP)
                handle_out0(dev);
        }
        else
        {
            if (sts & USB_DOEP0INT_SETUP)
            {
                USB->DOEP0INT = USB_DOEP0INT_SETUP;
                int supcnt = (USB->DOEP0TSIZ & 0x60000000UL) >> 29;
                supcnt = (supcnt == 3) ? 2 : supcnt;
                dev->dev_req = ep0_setup_pkt[2 - supcnt];
                usb_setup(dev);
            }
        }

        if (sts & USB_DOEP0INT_STSPHSERCVD)
            USB->DOEP0INT = USB_DOEP0INT_STSPHSERCVD;
    }
}

static int usb_core_init(void)
{
    const uint32_t total_rx_fifo_size = 128;
    const uint32_t ep_tx_fifo_size = 64;
    uint32_t address, depth;

    USB->ROUTE = USB_ROUTE_PHYPEN; /* Enable PHY pins.  */

    USB->PCGCCTL &= ~USB_PCGCCTL_STOPPCLK;
    USB->PCGCCTL &= ~(USB_PCGCCTL_PWRCLMP | USB_PCGCCTL_RSTPDWNMODULE);

    /* Core Soft Reset */
    {
        USB->GRSTCTL |= USB_GRSTCTL_CSFTRST;
        while (USB->GRSTCTL & USB_GRSTCTL_CSFTRST)
        {
        }

        /* Wait for AHB master IDLE state. */
        while (!(USB->GRSTCTL & USB_GRSTCTL_AHBIDLE))
        {
        }
    }

    /* Setup full speed device */
    USB->DCFG = (USB->DCFG & ~_USB_DCFG_DEVSPD_MASK) | USB_DCFG_DEVSPD_FS;

    /* Stall on non-zero len status OUT packets (ctrl transfers). */
    USB->DCFG |= USB_DCFG_NZSTSOUTHSHK;

    /* Set periodic frame interval to 80% */
    USB->DCFG &= ~_USB_DCFG_PERFRINT_MASK;

    USB->GAHBCFG = (USB->GAHBCFG & ~_USB_GAHBCFG_HBSTLEN_MASK) | USB_GAHBCFG_DMAEN | USB_GAHBCFG_HBSTLEN_SINGLE;

    /* Ignore frame numbers on ISO transfers. */
    USB->DCTL = (USB->DCTL & ~DCTL_WO_BITMASK) | USB_DCTL_IGNRFRMNUM;

    /* Set Rx FIFO size */
    USB->GRXFSIZ = total_rx_fifo_size;

    /* Set Tx EP0 FIFO size */
    address = total_rx_fifo_size;
    depth = ep_tx_fifo_size;
    USB->GNPTXFSIZ = (depth << 16 /*NPTXFINEPTXF0DEP*/) | address /*NPTXFSTADDR*/;

    efm32hg_connect();

    return 0;
}

void usb_init(void)
{
    // Follow section 14.3.2 USB Initialization, EFM32HG-RM.pdf
    struct efm32hg_rev rev;

    /* Select LFRCO as LFCCLK clock */
    CMU->LFCLKSEL = (CMU->LFCLKSEL & ~0x30UL) | (0x1 /* LFRCO */ << 4 /* LFC */);

    CMU->LFCCLKEN0 |= CMU_LFCCLKEN0_USBLE;

    /* Select USHFRCO as clock source for USB */
    CMU->OSCENCMD = CMU_OSCENCMD_USHFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_USHFRCORDY))
        ;

    /* Switch oscillator */
    CMU->CMD = CMU_CMD_USBCCLKSEL_USHFRCO;

    /* Wait until clock is activated */
    while ((CMU->STATUS & CMU_STATUS_USBCUSHFRCOSEL) == 0)
        ;

    /* Turn on Low Energy Mode (LEM) features. */
    efm32hg_revno(&rev);

    if ((rev.family == 5) && (rev.major == 1) && (rev.minor == 0))
    {
        /* First Happy Gecko chip revision did not have 
      all LEM features enabled. */
        USB->CTRL = USB_CTRL_LEMOSCCTRL_GATE | USB_CTRL_LEMIDLEEN | USB_CTRL_LEMPHYCTRL;
    }
    else
    {
        USB->CTRL = USB_CTRL_LEMOSCCTRL_GATE | USB_CTRL_LEMIDLEEN | USB_CTRL_LEMPHYCTRL | USB_CTRL_LEMNAKEN | USB_CTRL_LEMADDRMEN;
    }

    // enable interrupt in NVIC...
    NVIC_EnableIRQ(USB_IRQn);

    usb_core_init();

    efm32hg_set_daddr(0);

    /* Unmask interrupts for TX and RX */
    USB->GAHBCFG |= USB_GAHBCFG_GLBLINTRMSK;
    USB->GINTMSK = USB_GINTMSK_USBRSTMSK |
                   USB_GINTMSK_ENUMDONEMSK | USB_GINTMSK_IEPINTMSK | USB_GINTMSK_OEPINTMSK;
    USB->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
    USB->DOEPMSK = USB_DOEPMSK_SETUPMSK | USB_DOEPMSK_XFERCOMPLMSK | USB_DOEPMSK_STSPHSERCVDMSK;
    USB->DIEPMSK = USB_DIEPMSK_XFERCOMPLMSK;
    USB_DOUTEPS[0].CTL = USB_DOEP_CTL_SETD0PIDEF | USB_DOEP_CTL_USBACTEP | USB_DOEP_CTL_SNAK | USB_DOEP_CTL_EPTYPE_CONTROL;
    USB_DINEPS[0].CTL = USB_DIEP_CTL_SETD0PIDEF | USB_DIEP_CTL_USBACTEP | USB_DIEP_CTL_SNAK | USB_DIEP_CTL_EPTYPE_CONTROL;
}
