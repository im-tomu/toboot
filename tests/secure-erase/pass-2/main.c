#include "../../../toboot/toboot-api.h"
#include "../../../toboot/mcu.h"
#include <string.h>

extern uint8_t secure_data1;
static const uint8_t secure_data1_check[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab};

extern uint32_t secure_data2;
static const uint32_t secure_data2_check[] = {
    0xd08cb6ab,
    0x7b756095,
    0x53e3e240,
    0x0c81eb07,
    0xfa823033,
    0xf7c986e4,
    0x8406157a,
    0x75f53ea2,
    0x83cf1128,
    0x02f18051,
};

extern struct toboot_runtime toboot_runtime;
int failures = 0;

void Reset_Handler(void)
{
    failures = 0;
    CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
    CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO;
    CMU->HFPERCLKDIV = 1 << 8;

    // Mux things
    // Mux PA0 (Green LED)
    GPIO->P[0].MODEL &= ~_GPIO_P_MODEL_MODE0_MASK;
    GPIO->P[0].MODEL |= GPIO_P_MODEL_MODE0_WIREDAND;
    GPIO->P[0].DOUTCLR = (1 << 0);

    // Mux PB7 (Red LED)
    GPIO->P[1].MODEL &= ~_GPIO_P_MODEL_MODE7_MASK;
    GPIO->P[1].MODEL |= GPIO_P_MODEL_MODE7_WIREDAND;
    GPIO->P[1].DOUTCLR = (1 << 7);

    // Reset the boot counter, to prevent boot loops
    //toboot_runtime.boot_count = 0;

    // Disable the watchdog timer
    WDOG->CTRL = 0;

    unsigned int i;
    uint8_t *d1 = (uint32_t *)0x4400;
    for (i = 0; i < (sizeof(secure_data1_check) / sizeof(*secure_data1_check)); i++) {
        if (d1[i] != 0xff) {
            // Turn on the red LED
            GPIO->P[1].DOUTSET = (1 << 7);
            failures++;
        }
    }

    uint32_t *d2 = (uint32_t *)0x4800;//&secure_data2;
    for (i = 0; i < (sizeof(secure_data2_check) / sizeof(*secure_data2_check)); i++) {
        if (d2[i] != secure_data2_check[i]) {
            // Turn on the red LED
            GPIO->P[1].DOUTSET = (1 << 7);
            failures++;
        }
    }

    if (!failures) {
        // Turn on the green LED
        GPIO->P[0].DOUTSET = (1 << 0);
    }
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