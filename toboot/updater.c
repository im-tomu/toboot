#include "mcu.h"
#include "dfu.h"
#include "usb_dev.h"

bool fl_is_idle(void);

__attribute__((noreturn))
void updater(void) {
    usb_init();
    dfu_init();
    int i;

    while (dfu_getstate() != dfuMANIFEST)
        // Wait for firmware download
        watchdog_refresh();

    while (!fl_is_idle())
        watchdog_refresh();

    for (i = 0; i < 1075000; i++)
       watchdog_refresh();

    while(1);
}