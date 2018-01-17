#include "mcu.h"
#include "usb_dev.h"

__attribute__((noreturn))
void updater(void) {
    usb_init();
    volatile uint32_t *douttgl = &GPIO->P[1].DOUTTGL;

    while (1) {
        int i;
        //GPIO->P[1].DOUTTGL = (1 << 7);
        *douttgl = (1 << 7);
        for (i = 0; i < 1075000; i++)
            asm("nop");
    }
}