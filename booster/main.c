#include "booster.h"

#include "../toboot/mcu.h"
#include "../toboot/toboot-api.h"

// Defined in the linker
extern struct booster_data booster_data;

// Configure Toboot by identifying this as a V2 header.
// Place the header at page 16, to maintain compatibility with
// the serial bootloader and legacy programs.
__attribute__((used, section(".toboot_header"))) struct toboot_configuration toboot_configuration = {
    .magic = TOBOOT_V2_MAGIC,
    .start = 16,
    .config = TOBOOT_CONFIG_FLAG_ENABLE_IRQ,
};

__attribute__((section(".toboot_runtime"))) extern struct toboot_runtime toboot_runtime;

// Returns whether the flash controller is busy
static bool ftfl_busy()
{
    return (MSC->STATUS & MSC_STATUS_BUSY);
}

// Wait for the flash memory controller to finish any pending operation.
static void ftfl_busy_wait()
{
    while (ftfl_busy())
        ;
}

// Erase the sector that contains the specified address.
static void ftfl_begin_erase_sector(uint32_t address)
{
    // Erase the page at the specified address.
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;

    ftfl_busy_wait();
    MSC->ADDRB = address;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;
}

// Program the specified word at the given address.
// Assumes the page has already been erased.
void ftfl_begin_program_word(uint32_t address, uint32_t value)
{
    // Write the buffer word to the currently selected address.
    // Note that after this is done, the address is incremented by 4.
    ftfl_busy_wait();
    MSC->ADDRB = address;

    ftfl_busy_wait();
    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
    MSC->WDATA = value;
    MSC->WRITECMD = MSC_WRITECMD_WRITEONCE;
}

uint32_t bytes_left;
uint32_t target_addr;
uint32_t *current_ptr;
uint32_t page_offset;

// Erase the Toboot configuration, which will prevent us from
// overwriting Toboot again.
// Run this from RAM to ensure we don't hardfault.
// Reboot after we're done.
__attribute__((section(".data"), noreturn)) static void finish_flashing(void)
{
    uint32_t vector_addr = (uint32_t)&toboot_configuration;

    // Jump back into the bootloader when we reboot
    toboot_runtime.magic = TOBOOT_FORCE_ENTRY_MAGIC;

    while (MSC->STATUS & MSC_STATUS_BUSY)
        ;

    MSC->WRITECTRL |= MSC_WRITECTRL_WREN;
    MSC->ADDRB = vector_addr;
    MSC->WRITECMD = MSC_WRITECMD_LADDRIM;
    MSC->WRITECMD = MSC_WRITECMD_ERASEPAGE;

    while (MSC->STATUS & MSC_STATUS_BUSY)
        ;

    NVIC_SystemReset();
    while (1);
}

void Reset_Handler(void)
{
    // If the booster data is too big, just let the watchdog timer reboot us,
    // since the program is invalid.
    if (booster_data.payload_size > 65536)
    {
        NVIC_SystemReset();
    }

    // Ensure the hash matches what's expected.
    if (XXH32(booster_data.payload, booster_data.payload_size, BOOSTER_SEED) != booster_data.xxhash)
    {
        NVIC_SystemReset();
    }

    // Reset the boot counter, to prevent boot loops
    toboot_runtime.boot_count = 0;

    // Disable the watchdog timer
    WDOG->CTRL = 0;

    // Ensure interrupts are disabled, because this updater runs without them.
    __disable_irq();

    // Enable flash memory clocks
    CMU->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_AUXHFRCORDY))
        ;

    // Unlock the flash controller
    MSC->LOCK = MSC_UNLOCK_CODE;

    bytes_left = booster_data.payload_size;
    target_addr = 0;
    current_ptr = &booster_data.payload[0];

    while (bytes_left && (target_addr < (uint32_t)&toboot_configuration))
    {
        ftfl_begin_erase_sector(target_addr);

        for (page_offset = 0; page_offset < 1024; page_offset += 4)
        {
            ftfl_begin_program_word(target_addr, *current_ptr++);
            target_addr += 4;
        }

        if (bytes_left > 1024)
            bytes_left -= 1024;
        else
            bytes_left = 0;
    }

    finish_flashing();
}

void NMI_Handler(void)
{
    asm("bkpt #1");
    while (1)
        ;
}
void HardFault_Handler(void)
{
    asm("bkpt #2");
    while (1)
        ;
}

// The location of the stack, defined by the linker
extern uint32_t __stack;
__attribute__((used, section(".vectors"))) void *__vectors[] = {
    &__stack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
};
