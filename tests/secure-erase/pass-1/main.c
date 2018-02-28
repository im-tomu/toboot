#include "../../../toboot/toboot-api.h"
#include <string.h>

__attribute__((used, section(".secure1")))
static const uint8_t secure_data1[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab};

__attribute__((used, section(".secure2")))
static const uint32_t secure_data2[] = {0xd08cb6ab,0x7b756095,0x53e3e240,0x0c81eb07,0xfa823033,0xf7c986e4,0x8406157a,0x75f53ea2,0x83cf1128,0x02f18051,};

extern struct toboot_runtime toboot_runtime;

void Reset_Handler(void)
{
    memcpy((uint32_t *)0x20000008, secure_data1, 16);
    memcpy((uint32_t *)0x2000000c, secure_data2, 16);
    while(1);
}

// The location of the stack, defined by the linker
extern uint32_t __stack;
__attribute__((used, section(".vectors"))) void *__vectors[] = {
    &__stack,
    Reset_Handler,
};

// Configure Toboot by identifying this as a V2 header.
// Place the header at page 16, to maintain compatibility with
// the serial bootloader and legacy programs.
__attribute__((used, section(".toboot_header"))) struct toboot_configuration toboot_configuration = {
    .magic = TOBOOT_V2_MAGIC,
    .start = 16,
    .config = TOBOOT_CONFIG_FLAG_ENABLE_IRQ,
    .erase_mask_lo = (1 << 17), // secure_data1 only
};