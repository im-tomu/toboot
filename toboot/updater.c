#include "mcu.h"

__attribute__((noreturn))
void updater(void) {
    while (1) {
        //GPIO->P[0].DOUTTGL = (1 << 0);
        //GPIO->P[1].DOUTTGL = (1 << 7);
        //asm("bkpt #3");
    }
}