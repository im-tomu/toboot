#include "mcu.h"
#include "dfu.h"
#include "usb_dev.h"

__attribute__((noreturn))
void updater(void) {
    usb_init();
    dfu_init();
    volatile uint32_t *douttgl = &GPIO->P[1].DOUTTGL;
    int i;

    while (dfu_getstate() != dfuMANIFEST) {
        //GPIO->P[1].DOUTTGL = (1 << 7);
        *douttgl = (1 << 7);
        for (i = 0; i < 1075000; i++)
            asm("nop");
            
        // Wait for firmware download
        //while () {
        //    watchdog_refresh();
        //}
    }

    for (i = 0; i < 1075000; i++)
        asm("nop");
    *(uint32_t *)0xE000ED0C = 0x05FA0004;
    while(1);
}