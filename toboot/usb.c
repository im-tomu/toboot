#include "mcu.h"

void usbConnect(void) {
    USB->DCTL &= ~(DCTL_WO_BITMASK | USB_DCTL_SFTDISCON);
}

void usbDisconnect(void) {
    USB->DCTL = (USB->DCTL & ~DCTL_WO_BITMASK) | USB_DCTL_SFTDISCON;
}