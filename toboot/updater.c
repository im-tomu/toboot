#include "mcu.h"
#include "dfu.h"
#include "usb_dev.h"

bool fl_is_idle(void);

__attribute__((noreturn))
void updater(void) {
    usb_init();
    dfu_init();

    while (dfu_getstate() != dfuMANIFEST_WAIT_RESET)
        // Wait for firmware download
        watchdog_refresh();

    while (!fl_is_idle())
        watchdog_refresh();

    // Let the watchdog reset the system
    while(1);
}