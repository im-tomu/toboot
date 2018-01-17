/*
 * Simple control-only USB driver for DFU bootloader mode.
 * Originally based on:
 * 
 * Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2013 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows 
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "mcu.h"
#include "usb_dev.h"
#include "usb_desc.h"
#include "dfu.h"

#define STANDARD_ENDPOINT_DESC_SIZE 0x09
#define USB_MAX_PACKET_SIZE 64 /* For FS device */

/* endpoints enumeration */
#define ENDP0 ((uint8_t)0)
#define ENDP1 ((uint8_t)1)
#define ENDP2 ((uint8_t)2)
#define ENDP3 ((uint8_t)3)
#define ENDP4 ((uint8_t)4)
#define ENDP5 ((uint8_t)5)
#define ENDP6 ((uint8_t)6)
#define ENDP7 ((uint8_t)7)

volatile uint8_t usb_configuration = 0;

void *memcpy(void *dst, const void *src, size_t cnt);

enum RECIPIENT_TYPE
{
    DEVICE_RECIPIENT = 0, /* Recipient device    */
    INTERFACE_RECIPIENT,  /* Recipient interface */
    ENDPOINT_RECIPIENT,   /* Recipient endpoint  */
    OTHER_RECIPIENT
};

enum DESCRIPTOR_TYPE
{
    DEVICE_DESCRIPTOR = 1,
    CONFIG_DESCRIPTOR,
    STRING_DESCRIPTOR,
    INTERFACE_DESCRIPTOR,
    ENDPOINT_DESCRIPTOR
};

#define REQUEST_DIR 0x80      /* Mask to get request dir  */
#define REQUEST_TYPE 0x60     /* Mask to get request type */
#define STANDARD_REQUEST 0x00 /* Standard request         */
#define CLASS_REQUEST 0x20    /* Class request            */
#define VENDOR_REQUEST 0x40   /* Vendor request           */
#define RECIPIENT 0x1F        /* Mask to get recipient    */

static struct device_req ep0_setup_pkt[3] __attribute__((aligned(4)));
static char ctrl_send_buf[USB_MAX_PACKET_SIZE] __attribute__((aligned(4)));

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

static struct usb_dev default_dev;
struct usb_dev *dev = &default_dev;
static uint8_t reply_buffer[8];

#if 0

// buffer descriptor table
typedef struct
{
    uint32_t desc;
    void *addr;
} bdt_t;

__attribute__((section(".usbdescriptortable"), used)) static bdt_t table[64]; // BDT page (512 bytes)

#define BDT_OWN 0x80
#define BDT_DATA1 0x40
#define BDT_DATA0 0x00
#define BDT_DTS 0x08
#define BDT_STALL 0x04
#define BDT_PID(n) (((n) >> 2) & 15)

#define BDT_DESC(count, data) (BDT_OWN | BDT_DTS | ((data) ? BDT_DATA1 : BDT_DATA0) | ((count) << 16))

// No DTS, we want to accept either DATA0 or DATA1 for RX.
// This makes buffer management way less awful for EP0 OUT with
// multiple packets per transaction.
#define BDT_DESC_RX(count) (BDT_OWN | ((count) << 16))

#define TX 1
#define RX 0
#define ODD 1
#define EVEN 0
#define DATA0 0
#define DATA1 1
#define index(endpoint, tx, odd) (((endpoint) << 2) | ((tx) << 1) | (odd))
#define stat2bufferdescriptor(stat) (table + ((stat) >> 2))

static union {
    struct
    {
        union {
            struct
            {
                uint8_t bmRequestType;
                uint8_t bRequest;
            };
            uint16_t wRequestAndType;
        };
        uint16_t wValue;
        uint16_t wIndex;
        uint16_t wLength;
    };
    struct
    {
        uint32_t word1;
        uint32_t word2;
    };
} setup;

#define GET_STATUS 0
#define CLEAR_FEATURE 1
#define SET_FEATURE 3
#define SET_ADDRESS 5
#define GET_DESCRIPTOR 6
#define SET_DESCRIPTOR 7
#define GET_CONFIGURATION 8
#define SET_CONFIGURATION 9
#define GET_INTERFACE 10
#define SET_INTERFACE 11
#define SYNCH_FRAME 12

// SETUP always uses a DATA0 PID for the data field of the SETUP transaction.
// transactions in the data phase start with DATA1 and toggle (figure 8-12, USB1.1)
// Status stage uses a DATA1 PID.

static uint8_t ep0_rx0_buf[EP0_SIZE] __attribute__((aligned(4)));
static uint8_t ep0_rx1_buf[EP0_SIZE] __attribute__((aligned(4)));
static const uint8_t *ep0_tx_ptr = NULL;
static uint16_t ep0_tx_len;
static uint16_t ep0_rx_offset;
static uint8_t ep0_tx_bdt_bank = 0;
static uint8_t ep0_tx_data_toggle = 0;

static uint8_t reply_buffer[8];

volatile uint8_t usb_configuration = 0;

static void endpoint0_stall(void)
{
    USB0_ENDPT0 = USB_ENDPT_EPSTALL | USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
}

static void endpoint0_transmit(const void *data, uint32_t len)
{
    table[index(0, TX, ep0_tx_bdt_bank)].addr = (void *)data;
    table[index(0, TX, ep0_tx_bdt_bank)].desc = BDT_DESC(len, ep0_tx_data_toggle);
    ep0_tx_data_toggle ^= 1;
    ep0_tx_bdt_bank ^= 1;
}

void usb_isr(void)
{
    uint8_t status, stat;

restart:
    status = USB0_ISTAT;

    if ((status & USB_INTEN_SOFTOKEN /* 04 */))
    {
        // Clear SOF interrupt
        USB0_ISTAT = USB_INTEN_SOFTOKEN;
    }

    if ((status & USB_ISTAT_TOKDNE /* 08 */))
    {
        uint8_t endpoint;
        stat = USB0_STAT;
        endpoint = stat >> 4;
        if (endpoint == 0)
        {
            usb_control(stat);
        }
        USB0_ISTAT = USB_ISTAT_TOKDNE;
        goto restart;
    }

    if (status & USB_ISTAT_USBRST /* 01 */)
    {

        // initialize BDT toggle bits
        USB0_CTL = USB_CTL_ODDRST;
        ep0_tx_bdt_bank = 0;

        // set up buffers to receive Setup and OUT packets
        table[index(0, RX, EVEN)].desc = BDT_DESC_RX(EP0_SIZE);
        table[index(0, RX, EVEN)].addr = ep0_rx0_buf;
        table[index(0, RX, ODD)].desc = BDT_DESC_RX(EP0_SIZE);
        table[index(0, RX, ODD)].addr = ep0_rx1_buf;
        table[index(0, TX, EVEN)].desc = 0;
        table[index(0, TX, ODD)].desc = 0;

        // activate endpoint 0
        USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;

        // clear all ending interrupts
        USB0_ERRSTAT = 0xFF;
        USB0_ISTAT = 0xFF;

        // set the address to zero during enumeration
        USB0_ADDR = 0;

        // enable other interrupts
        USB0_ERREN = 0xFF;
        USB0_INTEN = USB_INTEN_TOKDNEEN |
                     USB_INTEN_SOFTOKEN |
                     USB_INTEN_STALLEN |
                     USB_INTEN_ERROREN |
                     USB_INTEN_USBRSTEN |
                     USB_INTEN_SLEEPEN;

        // is this necessary?
        USB0_CTL = USB_CTL_USBENSOFEN;
        return;
    }

    if ((status & USB_ISTAT_STALL /* 80 */))
    {
        USB0_ENDPT0 = USB_ENDPT_EPRXEN | USB_ENDPT_EPTXEN | USB_ENDPT_EPHSHK;
        USB0_ISTAT = USB_ISTAT_STALL;
    }

    if ((status & USB_ISTAT_ERROR /* 02 */))
    {
        uint8_t err = USB0_ERRSTAT;
        USB0_ERRSTAT = err;
        USB0_ISTAT = USB_ISTAT_ERROR;
    }

    if ((status & USB_ISTAT_SLEEP /* 10 */))
    {
        USB0_ISTAT = USB_ISTAT_SLEEP;
    }
}
#endif

void efm32hg_core_reset(void)
{
    USB->PCGCCTL &= ~USB_PCGCCTL_STOPPCLK;
    USB->PCGCCTL &= ~(USB_PCGCCTL_PWRCLMP | USB_PCGCCTL_RSTPDWNMODULE);

    /* Core Soft Reset */
    USB->GRSTCTL |= USB_GRSTCTL_CSFTRST;
    while (USB->GRSTCTL & USB_GRSTCTL_CSFTRST)
    {
    }

    /* Wait for AHB master IDLE state. */
    while (!(USB->GRSTCTL & USB_GRSTCTL_AHBIDLE))
    {
    }
}

static void
efm32hg_flush_rx_fifo(void)
{
    USB->GRSTCTL = USB_GRSTCTL_RXFFLSH;
    while (USB->GRSTCTL & USB_GRSTCTL_RXFFLSH)
    {
    }
}

static void
efm32hg_flush_tx_fifo(uint8_t n)
{
    USB->GRSTCTL = USB_GRSTCTL_TXFFLSH | (n << 6);
    while (USB->GRSTCTL & USB_GRSTCTL_TXFFLSH)
    {
    }
}

static void
efm32hg_enable_ints(void)
{
    /* Disable all interrupts. */
    USB->GINTMSK = 0;

    /* Clear pending interrupts */
    USB->GINTSTS = 0xFFFFFFFF;

    USB->GINTMSK = USB_GINTMSK_USBRSTMSK | USB_GINTMSK_ENUMDONEMSK | USB_GINTMSK_IEPINTMSK | USB_GINTMSK_OEPINTMSK;
}

static void
efm32hg_connect(void)
{
    USB->DCTL &= ~(DCTL_WO_BITMASK | USB_DCTL_SFTDISCON);
}

static void __attribute__((unused))
efm32hg_disconnect(void)
{
    USB->DCTL = (USB->DCTL & ~DCTL_WO_BITMASK) | USB_DCTL_SFTDISCON;
}

static void
efm32hg_set_daddr(uint8_t daddr)
{
    USB->DCFG = (USB->DCFG & ~_USB_DCFG_DEVADDR_MASK) | (daddr << 4);
}

static void
efm32hg_prepare_ep0_setup(struct usb_dev *dev)
{
    (void)dev;
    USB->DOEP0TSIZ = (8 * 3 << 0) /* XFERSIZE */
                     | (1 << 19)  /* PKTCNT */
                     | (3 << 29); /* SUPCNT */
    USB->DOEP0DMAADDR = (uint32_t)&ep0_setup_pkt;
    USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DOEP0CTL_EPENA;
}

static void
efm32hg_prepare_ep0_out(const void *buf, size_t len, uint32_t ep0mps)
{
    USB->DOEP0DMAADDR = (uint32_t)buf;
    USB->DOEP0TSIZ = (len << 0)   /* XFERSIZE */
                     | (1 << 19); /* PKTCNT */
    USB->DOEP0CTL = (USB->DOEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DOEP0CTL_CNAK | USB_DOEP0CTL_EPENA | ep0mps;
}

static void
efm32hg_prepare_ep0_in(const void *buf, size_t len, uint32_t ep0mps)
{
    USB->DIEP0DMAADDR = (uint32_t)buf;
    USB->DIEP0TSIZ = (len << 0)   /* XFERSIZE */
                     | (1 << 19); /* PKTCNT */
    USB->DIEP0CTL = (USB->DIEP0CTL & ~DEPCTL_WO_BITMASK) | USB_DIEP0CTL_CNAK | USB_DIEP0CTL_EPENA | ep0mps;
}

static void
efm32hg_ep_out_stall(uint8_t n)
{
    uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_BITMASK;
    uint32_t eptype = ctl & 0xC0000UL;

    if (eptype != USB_DOEP_CTL_EPTYPE_ISO)
    {
        ctl |= USB_DIEP_CTL_STALL;
        USB_DOUTEPS[n].CTL = ctl;
    }
}

static void
efm32hg_ep_in_stall(uint8_t n)
{
    uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_BITMASK;
    uint32_t eptype = ctl & 0xC0000UL;

    if (eptype != USB_DIEP_CTL_EPTYPE_ISO)
    {
        USB_DINEPS[n].CTL |= USB_DIEP_CTL_STALL;
        if (ctl & USB_DIEP_CTL_EPENA)
            ctl |= USB_DIEP_CTL_EPDIS;
        USB_DINEPS[n].CTL = ctl;
    }
}
/*
static void
efm32hg_ep_out_unstall(uint8_t n)
{
    uint32_t ctl = USB_DOUTEPS[n].CTL & ~DEPCTL_WO_BITMASK;
    uint32_t eptype = ctl & 0xC0000UL;

    if (eptype == USB_DOEP_CTL_EPTYPE_INT || eptype == USB_DOEP_CTL_EPTYPE_BULK)
    {
        ctl |= USB_DOEP_CTL_SETD0PIDEF;
        ctl &= ~USB_DOEP_CTL_STALL;
        USB_DOUTEPS[n].CTL = ctl;
    }
}

static void
efm32hg_ep_in_unstall(uint8_t n)
{
    uint32_t ctl = USB_DINEPS[n].CTL & ~DEPCTL_WO_BITMASK;
    uint32_t eptype = ctl & 0xC0000UL;

    if (eptype == USB_DIEP_CTL_EPTYPE_INT || eptype == USB_DIEP_CTL_EPTYPE_BULK)
    {
        ctl |= USB_DIEP_CTL_SETD0PIDEF;
        ctl &= ~USB_DIEP_CTL_STALL;
        USB_DINEPS[n].CTL = ctl;
    }
}

static int
efm32hg_ep_in_is_stall(uint8_t n)
{
    return (USB_DINEPS[n].CTL & USB_DIEP_CTL_STALL) ? 1 : 0;
}

static int
efm32hg_ep_out_is_stall(uint8_t n)
{
    return (USB_DOUTEPS[n].CTL & USB_DOEP_CTL_STALL) ? 1 : 0;
}

static int
efm32hg_ep_in_is_disabled(uint8_t n)
{
    return (USB_DINEPS[n].CTL & USB_DIEP_CTL_EPENA) ? 0 : 1;
}

static int
efm32hg_ep_out_is_disabled(uint8_t n)
{
    return (USB_DOUTEPS[n].CTL & USB_DOEP_CTL_EPENA) ? 0 : 1;
}
*/

static void
handle_datastage_out(struct usb_dev *dev)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len = USB->DOEP0TSIZ & 0x7FUL; /* XFERSIZE */
    uint32_t pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);

    data_p->len -= len;
    data_p->addr += len;

    len = data_p->len < pktsize ? data_p->len : pktsize;

    if (data_p->len == 0)
    {
        /* No more data to receive, proceed to send acknowledge for IN.  */
        efm32hg_prepare_ep0_setup(dev);
        dev->state = WAIT_STATUS_IN;
        efm32hg_prepare_ep0_in(NULL, 0, 0);
    }
    else
    {
        dev->state = OUT_DATA;
        efm32hg_prepare_ep0_out(data_p->addr, len, 0);
    }
}

static void
handle_datastage_in(struct usb_dev *dev)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len = 64 >> (USB->DOEP0CTL & 0x3UL);

    if ((data_p->len == 0) && (dev->state == LAST_IN_DATA))
    {
        if (data_p->require_zlp)
        {
            data_p->require_zlp = 0;

            efm32hg_prepare_ep0_setup(dev);
            /* No more data to send.  Send empty packet */
            efm32hg_prepare_ep0_in(NULL, 0, 0);
        }
        else
        {
            /* No more data to send, proceed to receive OUT acknowledge.  */
            dev->state = WAIT_STATUS_OUT;
            efm32hg_prepare_ep0_out(NULL, 0, 0);
        }

        return;
    }

    dev->state = (data_p->len <= len) ? LAST_IN_DATA : IN_DATA;

    if (len > data_p->len)
        len = data_p->len;

    efm32hg_prepare_ep0_setup(dev);
    efm32hg_prepare_ep0_in(data_p->addr, len, 0);
    data_p->len -= len;
    data_p->addr += len;
}

int usb_lld_ctrl_ack(struct usb_dev *dev)
{
    /* Zero length packet for ACK.  */
    efm32hg_prepare_ep0_setup(dev);
    dev->state = WAIT_STATUS_IN;
    efm32hg_prepare_ep0_in(NULL, 0, 0);
    return 0;
}

/*
 * BUF: Pointer to data memory.  Data memory should not be allocated
 *      on stack when BUFLEN > USB_MAX_PACKET_SIZE.
 *
 * BUFLEN: size of the data.
 */
int usb_lld_ctrl_send(struct usb_dev *dev, const void *buf, size_t buflen)
{
    struct ctrl_data *data_p = &dev->ctrl_data;
    uint32_t len_asked = dev->dev_req.wLength;
    uint32_t len;
    uint32_t pktsize = 64 >> (USB->DIEP0CTL & 0x3UL);

    data_p->addr = (void *)buf;
    data_p->len = buflen;

    /* Restrict the data length to be the one host asks for */
    if (data_p->len > len_asked)
        data_p->len = len_asked;

    data_p->require_zlp = (data_p->len != 0 && (data_p->len % pktsize) == 0);

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

    efm32hg_prepare_ep0_in(data_p->addr, len, 0);

    data_p->len -= len;
    data_p->addr += len;

    return 0 /*USB_EVENT_OK*/;
}

void usb_lld_ctrl_error(struct usb_dev *dev)
{
    dev->state = STALLED;
    efm32hg_ep_out_stall(ENDP0);
    efm32hg_ep_in_stall(ENDP0);
    dev->state = WAIT_SETUP;
    efm32hg_prepare_ep0_setup(dev);
}

static int
handle_out0(struct usb_dev *dev)
{
    if (dev->state == OUT_DATA)
        /* It's normal control WRITE transfer.  */
        handle_datastage_out(dev);
    else if (dev->state == WAIT_STATUS_OUT)
    { /* Control READ transfer done successfully.  */
        efm32hg_prepare_ep0_setup(dev);
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
        efm32hg_ep_out_stall(ENDP0);
        efm32hg_ep_in_stall(ENDP0);
        dev->state = WAIT_SETUP;
        efm32hg_prepare_ep0_setup(dev);
    }
    return 0 /*USB_EVENT_OK*/;
}

static void usb_setup(struct usb_dev *dev)
{
    const uint8_t *data = NULL;
    uint32_t datalen = 0;
    const usb_descriptor_list_t *list;

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

static int
handle_in0(struct usb_dev *dev)
{
    if (dev->state == IN_DATA || dev->state == LAST_IN_DATA)
        handle_datastage_in(dev);
    else if (dev->state == WAIT_STATUS_IN)
    { /* Control WRITE transfer done successfully.  */
        efm32hg_prepare_ep0_setup(dev);
        dev->state = WAIT_SETUP;
        return 1; //USB_EVENT_CTRL_WRITE_FINISH;
    }
    else
    {
        dev->state = STALLED;
        efm32hg_ep_out_stall(ENDP0);
        efm32hg_ep_in_stall(ENDP0);
        dev->state = WAIT_SETUP;
        efm32hg_prepare_ep0_setup(dev);
    }

    return 0; //USB_EVENT_OK;
}

void USB_Handler(void)
{
    uint32_t intsts = USB->GINTSTS & USB->GINTMSK;
    uint8_t ep;
    //    int r = USB_EVENT_OK;

    if (intsts & USB_GINTSTS_USBRST)
    {
        USB->GINTSTS = USB_GINTSTS_USBRST;
        //        return USB_MAKE_EV(USB_EVENT_DEVICE_RESET);
        return;
    }

    if (intsts & USB_GINTSTS_ENUMDONE)
    {
        USB->GINTSTS = USB_GINTSTS_ENUMDONE;
        efm32hg_prepare_ep0_setup(dev);
        efm32hg_enable_ints();
        dev->state = WAIT_SETUP;
    }

    if (intsts & USB_GINTSTS_IEPINT)
    {
        uint32_t epint = USB->DAINT & USB->DAINTMSK;

        ep = 0;
        while (epint != 0)
        {
            if (epint & 1)
            {
                uint32_t sts = USB_DINEPS[ep].INT & USB->DIEPMSK;
                if (sts & USB_DIEP_INT_XFERCOMPL)
                {
                    USB_DINEPS[ep].INT = USB_DIEP_INT_XFERCOMPL;
                    if (ep == 0)
                        handle_in0(dev);
                    else
                    {
                        // Unsupported in this bootloader.
                        asm("bkpt #99");
                        //                    else
                        //                    {
                        //                        len = USB_DINEPS[ep].TSIZ & 0x7FFFFUL; /* XFERSIZE */
                        //                        if (USB_DINEPS[ep].INT & USB_DIEP_INT_NAKINTRPT)
                        //                            USB_DINEPS[ep].INT = USB_DIEP_INT_NAKINTRPT;
                        //                        return USB_MAKE_TXRX(ep, 1, len);
                        //    return;
                    }
                }
            }
            ep++;
            epint >>= 1;
        }
    }

    if (intsts & USB_GINTSTS_OEPINT)
    {
        uint32_t epint = (USB->DAINT & USB->DAINTMSK) >> 16;

        ep = 0;
        while (epint != 0)
        {
            if (epint & 1)
            {
                uint32_t sts = USB_DOUTEPS[ep].INT & USB->DOEPMSK;

                if (ep == 0 && sts & USB_DOEP0INT_STUPPKTRCVD)
                {
                    USB_DOUTEPS[ep].INT = USB_DOEP0INT_STUPPKTRCVD;
                    sts &= ~USB_DOEP_INT_XFERCOMPL;
                }

                if (sts & USB_DOEP_INT_XFERCOMPL)
                {
                    USB_DOUTEPS[ep].INT = USB_DOEP_INT_XFERCOMPL;
                    if (ep == 0)
                    {
                        sts = USB->DOEP0INT & USB->DOEPMSK;

                        USB_DOUTEPS[ep].INT = USB_DOEP0INT_STUPPKTRCVD;

                        if (sts & USB_DOEP0INT_SETUP)
                        {
                            USB->DOEP0INT = USB_DOEP0INT_SETUP;
                            int supcnt = (USB->DOEP0TSIZ & 0x60000000UL) >> 29;
                            supcnt = (supcnt == 3) ? 2 : supcnt;
                            dev->dev_req = ep0_setup_pkt[2 - supcnt];
                            usb_setup(dev);
                        }
                        else if (dev->state != WAIT_SETUP)
                            handle_out0(dev);
                    }
                    else /* ep != 0 */
                    {
                        asm("bkpt #98");
                        //uint32_t remain = USB_DOUTEPS[ep].TSIZ & 0x7FFFFUL; /* XFERSIZE */
                        //if (remain < oeps[ep].xfersize)
                        //    len = oeps[ep].xfersize - remain;
                        //else
                        //    len = 0;
                        //return USB_MAKE_TXRX(ep, 0, len);
                        return;
                    }
                }
                else
                {
                    if (ep == 0 && sts & USB_DOEP0INT_SETUP)
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
            ep++;
            epint >>= 1;
        }
    }

    //return USB_MAKE_EV(r);
}

static int usb_core_init(void)
{
    const uint32_t total_rx_fifo_size = 128;
    const uint32_t total_tx_fifo_size = 256;
    const uint32_t ep_tx_fifo_size = 64;
    uint32_t address, depth;
    uint8_t ep;

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

    /* Set Tx EP FIFO sizes for all IN ep's */
    for (ep = 1; ep <= MAX_NUM_IN_EPS; ep++)
    {
        address += depth;
        depth = ep_tx_fifo_size;
        USB_DIEPTXFS[ep - 1] = (depth << 16)          /* INEPNTXFDEP */
                               | (address & 0x7FFUL); /* INEPNTXFSTADDR */
    }

    if (total_rx_fifo_size + total_tx_fifo_size > MAX_DEVICE_FIFO_SIZE_INWORDS)
        return -1;

    if (address > MAX_DEVICE_FIFO_SIZE_INWORDS)
        return -1;

    /* Flush the FIFO's */
    efm32hg_flush_tx_fifo(0x10); /* All Tx FIFO's */
    efm32hg_flush_rx_fifo();     /* The Rx FIFO   */

    /* Disable all device interrupts */
    USB->DIEPMSK = 0;
    USB->DOEPMSK = 0;
    USB->DAINTMSK = 0;
    USB->DIEPEMPMSK = 0;

    /* Disable all EP's, clear all EP ints. */
    for (ep = 0; ep <= MAX_NUM_IN_EPS; ep++)
    {
        USB_DINEPS[ep].CTL = 0;
        USB_DINEPS[ep].TSIZ = 0;
        USB_DINEPS[ep].INT = 0xFFFFFFFF;
    }

    for (ep = 0; ep <= MAX_NUM_OUT_EPS; ep++)
    {
        USB_DOUTEPS[ep].CTL = 0;
        USB_DOUTEPS[ep].TSIZ = 0;
        USB_DOUTEPS[ep].INT = 0xFFFFFFFF;
    }

    efm32hg_connect();

    return 0;
}

void usb_init(void)
{
    // Follow section 14.3.2 USB Initialization, EFM32HG-RM.pdf
    struct efm32hg_rev rev;

    /* Ensure selected oscillator is enabled, waiting for it to stabilize */
    CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_LFRCORDY))
        ;

    /* Select LFRCO as LFCCLK clock */
    CMU->LFCLKSEL = (CMU->LFCLKSEL & ~0x30UL) | (0x1 /* LFRCO */ << 4 /* LFC */);

    CMU->LFCCLKEN0 |= CMU_LFCCLKEN0_USBLE;

    /* Enable USB clock */
    CMU->HFCORECLKEN0 |= CMU_HFCORECLKEN0_USB | CMU_HFCORECLKEN0_USBC;

    CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_48MHZ;

    /* Select USHFRCO as clock source for USB */
    CMU->OSCENCMD = CMU_OSCENCMD_USHFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_USHFRCORDY))
        ;

    /* Switch oscillator */
    CMU->CMD = CMU_CMD_USBCCLKSEL_USHFRCO;

    /* Wait until clock is activated */
    while ((CMU->STATUS & CMU_STATUS_USBCUSHFRCOSEL) == 0)
        ;

    /* Enable USHFRCO Clock Recovery mode. */
    CMU->USBCRCTRL |= CMU_USBCRCTRL_EN;

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
    USB->GINTMSK = /*USB_GINTMSK_USBSUSPMSK
                 |*/ USB_GINTMSK_USBRSTMSK |
                   USB_GINTMSK_ENUMDONEMSK | USB_GINTMSK_IEPINTMSK | USB_GINTMSK_OEPINTMSK
        /*| USB_GINTMSK_WKUPINTMSK*/;
    USB->DAINTMSK = USB_DAINTMSK_INEPMSK0 | USB_DAINTMSK_OUTEPMSK0;
    USB->DOEPMSK = USB_DOEPMSK_SETUPMSK | USB_DOEPMSK_XFERCOMPLMSK | USB_DOEPMSK_STSPHSERCVDMSK;
    USB->DIEPMSK = USB_DIEPMSK_XFERCOMPLMSK;
}