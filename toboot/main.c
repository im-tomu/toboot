#include <stdint.h>
#include "toboot-api.h"
#include "toboot-internal.h"
#include "mcu.h"
#include "usb_desc.h"
#include "board.h"

#define AUTOBAUD_TIMER_CLOCK CMU_HFPERCLKEN0_TIMER0
#define BOOTLOADER_USART_CLOCKEN 0

#define RTC_INTERVAL_MSEC 250

static uint32_t *app_vectors;
enum bootloader_reason bootloader_reason;
__attribute__((noreturn)) void updater(void);

void RTC_Handler(void)
{
    // Clear interrupt flag
    RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;

#ifdef LED1_PORT    
    // Toggle the green LED
    GPIO->P[LED1_PORT].DOUTTGL = (1 << LED1_PIN);
#endif

#ifdef LED0_PORT    
    // Also toggle the red LED, to make a pattern of flashing lights.
    GPIO->P[LED0_PORT].DOUTTGL = (1 << LED0_PIN);
#endif    
}

/**************************************************************************/ /**
 * Initialize and start RTC.
 *****************************************************************************/
static void start_rtc(void)
{
    // Enable LFRCO for RTC
    CMU->OSCENCMD = CMU_OSCENCMD_LFRCOEN;
    // Enable RTC
    CMU->LFACLKEN0 = CMU_LFACLKEN0_RTC;

    // Clear interrupt flags
    RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;
    // 250 ms wakeup time
    RTC->COMP0 = (RTC_INTERVAL_MSEC * EFM32_LFRCO_FREQ) / 1000;
    // Enable Interrupts on COMP0
    RTC->IEN = RTC_IEN_COMP0;
    // Enable RTC interrupts
    NVIC_EnableIRQ(RTC_IRQn);
    // Enable RTC
    RTC->CTRL = RTC_CTRL_COMP0TOP | RTC_CTRL_DEBUGRUN | RTC_CTRL_EN;
}

static void gpio_set_mode(uint8_t port, uint8_t pin, uint8_t mode)
{
    volatile uint32_t* mode_reg = &(&GPIO->P[port].MODEL)[pin >> 3];
    uint8_t mode_offset = (pin & 7) << 2;
    *mode_reg = (*mode_reg & ~(0xf << mode_offset)) | (mode << mode_offset);
}

void __early_init(void)
{
    // Enable peripheral clocks.
    CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
    CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO | BOOTLOADER_USART_CLOCKEN | AUTOBAUD_TIMER_CLOCK;
    CMU->HFPERCLKDIV = 1 << 8;

    // Enable LE and DMA interfaces.
    CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE | CMU_HFCORECLKEN0_DMA | CMU_HFCORECLKEN0_USB | CMU_HFCORECLKEN0_USBC;

    // Setup LFA/LFB clock sources.
    CMU->LFCLKSEL = CMU_LFCLKSEL_LFA_LFRCO | CMU_LFCLKSEL_LFB_HFCORECLKLEDIV2;

    // Set the main CPU clock to run from USB
    CMU->LFCLKSEL = (CMU->LFCLKSEL & ~_CMU_LFCLKSEL_LFC_MASK) | CMU_LFCLKSEL_LFC_LFRCO;
    CMU->LFCCLKEN0 |= CMU_LFCCLKEN0_USBLE;

    // Calibrate USB based on communications
    CMU->USHFRCOCONF = CMU_USHFRCOCONF_BAND_48MHZ;

    // Enable USHFRCO Clock Recovery mode.
    CMU->USBCRCTRL |= CMU_USBCRCTRL_EN;

    // Turn on Low Energy Mode (LEM) features.
    USB->CTRL |= USB_CTRL_LEMIDLEEN | USB_CTRL_LEMPHYCTRL;

    /* Set HFRCO freq 21 MHz */
    CMU->HFRCOCTRL = (4 << 8) | (DEVINFO->HFRCOCAL1 & 0xff << 0);

    CMU->OSCENCMD = CMU_OSCENCMD_HFRCOEN;
    while (!(CMU->STATUS & CMU_STATUS_HFRCORDY))
        ;

    CMU->CMD |= CMU_CMD_HFCLKSEL_HFRCO;
    while ((CMU->STATUS & CMU_STATUS_HFRCOSEL) == 0)
        ;

    // Mux things
#ifdef LED1_PORT
    // Mux Green LED
    gpio_set_mode(LED1_PORT, LED1_PIN, GPIO_P_MODEL_MODE0_WIREDAND);
    GPIO->P[LED1_PORT].DOUTSET = (1 << LED1_PIN);
#endif
#ifdef LED0_PORT    
    // Mux Red LED
    gpio_set_mode(LED0_PORT, LED0_PIN, GPIO_P_MODEL_MODE0_WIREDAND);
    GPIO->P[LED0_PORT].DOUTCLR = (1 << LED0_PIN);
#endif

    // Set up the watchdog to reboot us after 15 ms.
    WDOG->CTRL |= WDOG_CTRL_EN;
    while (WDOG->SYNCBUSY & _WDOG_SYNCBUSY_MASK)
        ;

    // A value of 0 causes a reboot in 9 ticks.  The ticks
    // are at 1 kHz intervals, so this will reset the system
    // in about 9 seconds.
    WDOG->CTRL = WDOG_CTRL_CLKSEL_ULFRCO | WDOG_CTRL_EN | (3 << _WDOG_CTRL_PERSEL_SHIFT);

    start_rtc();
}

static int test_boot_token(const struct toboot_configuration *cfg)
{
    (void)cfg;
    // If we find a valid boot token in RAM, the application is asking us explicitly
    // to enter DFU mode. This is used to implement the DFU_DETACH command when the app
    // is running.

    return boot_token.magic == TOBOOT_FORCE_ENTRY_MAGIC;
}

static void busy_wait(int count)
{
    int i;
    for (i = 0; i < count + 20000; i++)
        asm("nop");
}


#define READ_CAP0B() (GPIO->P[BUTTON_SENSE_PORT].DIN & (1 << BUTTON_SENSE_PIN))
#define TOGGLE_CAP1A() (GPIO->P[BUTTON_DRIVE_PORT].DOUTTGL = (1 << BUTTON_DRIVE_PIN))

int test_pin_short(const struct toboot_configuration *cfg)
{
    int samples[4];

    // If the outer pins on the edge connector are shorted, enter the bootloader.
    // We want to test CAP1A (PC1) and CAP0B (PE12).

    // Test if thos entry method has been locked out.
    if (cfg->lock_entry == TOBOOT_LOCKOUT_MAGIC)
    {
        return 0;
    }

    gpio_set_mode(BUTTON_DRIVE_PORT, BUTTON_DRIVE_PIN, GPIO_P_MODEL_MODE0_PUSHPULL);
    gpio_set_mode(BUTTON_SENSE_PORT, BUTTON_SENSE_PIN, GPIO_P_MODEL_MODE0_INPUTPULL);
    busy_wait(20);

    GPIO->P[BUTTON_DRIVE_PORT].DOUTSET = (1 << BUTTON_DRIVE_PIN);
    busy_wait(15);
    samples[0] = READ_CAP0B();

    TOGGLE_CAP1A();
    busy_wait(15);
    samples[1] = READ_CAP0B();

    TOGGLE_CAP1A();
    busy_wait(15);
    samples[2] = READ_CAP0B();

    TOGGLE_CAP1A();
    busy_wait(15);
    samples[3] = READ_CAP0B();

    if (samples[0] && (!samples[1]) && samples[2] && (!samples[3]))
        return 1;

    return 0;
}

static int test_reset_cause(const struct toboot_configuration *cfg)
{
    int result = 0;
    int rstcause = RMU->RSTCAUSE;
    boot_token.reserved++;
    if (rstcause & RMU_RSTCAUSE_PORST)
    {
        boot_token.magic = 0;
        boot_token.boot_count = 0;
        boot_token.board_model = TOBOOT_BOARD;
        boot_token.reserved = 0;

        // Add a brief delay, to allow the decoupling caps time to charge.
        // If we don't pause here on first boot, then subsequent high-current
        // operations such as USB might cause the chip to brownout and reset,
        // leading to automatic booting of the application when first inserted.
        busy_wait(200);

        // If the user has requested that we enter the program at poweron,
        // then do so.
        if (!(cfg->config & TOBOOT_CONFIG_FLAG_AUTORUN))
            result = 1;
    }

    // Reset the "RSTCAUSE" value (EFM32HG-RM 9.3.1)
    RMU->CMD |= RMU_CMD_RCCLR;
    EMU->AUXCTRL |= EMU_CTRL_EMVREG;
    EMU->AUXCTRL &= ~EMU_CTRL_EMVREG;

    return result;
}

static int test_boot_failures(const struct toboot_configuration *cfg)
{
    (void)cfg;
    return boot_token.boot_count >= 3;
}

static int test_application_invalid(const struct toboot_configuration *cfg)
{
    extern uint32_t __ram_start__;
    extern uint32_t __ram_end__;
    extern uint32_t __bl_end__;
    extern uint32_t __app_end__;

    (void)cfg;
    // Make sure the stack pointer is in RAM.
    if (app_vectors[0] < (uint32_t)&__ram_start__)
        return 1;
    if (app_vectors[0] > (uint32_t)&__ram_end__)
        return 1;

    // Make sure the entrypoint is in flash, after Toboot
    if (app_vectors[1] < (uint32_t)&__bl_end__)
        return 1;
    if (app_vectors[1] >= (uint32_t)&__app_end__)
        return 1;

    return 0;
}

static int should_enter_bootloader(const struct toboot_configuration *cfg)
{
    // Reset the boot token if we've just been powered up for the first time
    if (test_reset_cause(cfg))
    {
        bootloader_reason = COLD_BOOT_CONFIGURATION_FLAG;
        return 1;
    }

    // If the special magic number is present, enter the bootloader
    if (test_boot_token(cfg))
    {
        bootloader_reason = BOOT_TOKEN_PRESENT;
        return 1;
    }

    // If the user is holding the button down
    if (test_pin_short(cfg))
    {
        bootloader_reason = BUTTON_HELD_DOWN;
        return 1;
    }

    // If we've failed to boot many times, enter the bootloader
    if (test_boot_failures(cfg))
    {
        bootloader_reason = BOOT_FAILED_TOO_MANY_TIMES;
        return 1;
    }

    // If there is no valid program, enter the bootloader
    if (test_application_invalid(cfg))
    {
        bootloader_reason = NO_PROGRAM_PRESENT;
        return 1;
    }

    bootloader_reason = NOT_ENTERING_BOOTLOADER;
    return 0;
}

__attribute__((noreturn)) static void boot_app(void)
{
    // Relocate IVT to application flash
    __disable_irq();
    NVIC->ICER[0] = 0xFFFFFFFF;
    SCB->VTOR = (uint32_t)&app_vectors[0];

    // Disable USB
    USB->GAHBCFG = _USB_GAHBCFG_RESETVALUE;
    USB->GINTMSK = _USB_GINTMSK_RESETVALUE;
    USB->DAINTMSK = _USB_DAINTMSK_RESETVALUE;
    USB->DOEPMSK = _USB_DOEPMSK_RESETVALUE;
    USB->DIEPMSK = _USB_DIEPMSK_RESETVALUE;
    USB->CTRL = _USB_CTRL_RESETVALUE;

    // Reset RTC settings.
    RTC->IEN = _RTC_IEN_RESETVALUE;
    RTC->IFC = _RTC_IFC_MASK;
    RTC->COMP0 = _RTC_COMP0_RESETVALUE;
    RTC->CTRL = _RTC_CTRL_RESETVALUE;

    // Wait for LF peripheral syncronization.
    while (RTC->SYNCBUSY & _RTC_SYNCBUSY_MASK)
        ;
    while (CMU->SYNCBUSY & CMU_SYNCBUSY_LFACLKEN0)
        ;

    /* Switch back to HFRCO default freq (14 MHz) */
    CMU->HFRCOCTRL = (3 << 8) | ((DEVINFO->HFRCOCAL0 >> 24) & 0xff);

    CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS | CMU_OSCENCMD_AUXHFRCODIS | CMU_OSCENCMD_LFRCODIS | CMU_OSCENCMD_USHFRCODIS;
    CMU->USBCRCTRL = _CMU_USBCRCTRL_RESETVALUE;

    GPIO->P[LED0_PORT].MODEL = _GPIO_P_MODEL_RESETVALUE;
    GPIO->P[LED1_PORT].MODEL = _GPIO_P_MODEL_RESETVALUE;
    GPIO->P[BUTTON_SENSE_PORT].MODEH = _GPIO_P_MODEL_RESETVALUE;
    GPIO->P[BUTTON_DRIVE_PORT].MODEL = _GPIO_P_MODEL_RESETVALUE;

    // Reset clock registers used
    CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE;
    CMU->HFPERCLKDIV = _CMU_HFPERCLKDIV_RESETVALUE;
    CMU->HFPERCLKEN0 = _CMU_HFPERCLKEN0_RESETVALUE;
    CMU->LFCLKSEL = _CMU_LFCLKSEL_RESETVALUE;
    CMU->LFACLKEN0 = _CMU_LFACLKEN0_RESETVALUE;

    // Refresh watchdog right before launching app
    watchdog_refresh();

    // Clear the boot token, so we don't repeatedly enter DFU mode.
    boot_token.magic = 0;
    boot_token.boot_count++;

    __enable_irq();

    asm volatile(
        "mov lr, %0 \n\t"
        "mov sp, %1 \n\t"
        "bx %2 \n\t"
        :
        : "r"(0xFFFFFFFF),
          "r"(app_vectors[0]),
          "r"(app_vectors[1]));
    while (1)
        ;
}

__attribute__((noreturn)) void bootloader_main(void)
{
    const struct toboot_configuration *cfg = tb_get_config();
    app_vectors = (uint32_t *)(1024 * cfg->start);

    if (should_enter_bootloader(cfg))
    {
        boot_token.magic = 0;
        boot_token.boot_count = 0;

#ifdef REASON_OFFSET
        // Update the iProduct field to reflect the bootloader reason,
        // which is described in a specialized product string.
        extern struct usb_string_descriptor_struct usb_string_product_name;
        usb_string_product_name.wString[REASON_OFFSET] += bootloader_reason;
#endif // REASON_OFFSET

        updater();
    }

    boot_app();
}
