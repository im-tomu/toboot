#include "../toboot/toboot-api.h"

__attribute__((section(".toboot_runtime")))
extern struct toboot_runtime toboot_runtime;

void Reset_Handler(void) {
    // Reset the boot counter, to prevent boot loops
    toboot_runtime.boot_count = 0;

    // Disable the watchdog timer
    *(uint32_t *)0x40088000UL = 0;

    while(1) {
    }
}

// The location of the stack, defined by the linker
extern uint32_t __stack;
__attribute__((used, section(".vectors")))
void *__vectors[] = {
    &__stack,
    Reset_Handler,
};

// Configure Toboot by identifying this as a V2 header.
// Place the header at page 16, to maintain compatibility with
// the serial bootloader and legacy programs.
__attribute__((used, section(".toboot_header")))
struct toboot_configuration toboot_configuration = {
    .magic = TOBOOT_V2_MAGIC,
    .start = 16,
    .config = TOBOOT_CONFIG_FLAG_ENABLE_IRQ,
};