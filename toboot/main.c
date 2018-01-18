#include <stdint.h>
#include "toboot.h"
#include "mcu.h"

#define AUTOBAUD_TIMER_CLOCK CMU_HFPERCLKEN0_TIMER0
#define BOOTLOADER_USART_CLOCKEN 0

#define RTC_INTERVAL_MSEC 250

static __attribute__((section(".appvectors"))) uint32_t appVectors[64];
enum bootloader_reason bootloader_reason;
__attribute__((noreturn)) void updater(void);

void RTC_Handler(void)
{
    // Clear interrupt flag
    RTC->IFC = RTC_IFC_COMP1 | RTC_IFC_COMP0 | RTC_IFC_OF;

    // Toggle the green LED
    GPIO->P[0].DOUTTGL = (1 << 0);
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

void __early_init(void)
{
    // Enable peripheral clocks.
    CMU->HFPERCLKDIV = CMU_HFPERCLKDIV_HFPERCLKEN;
    CMU->HFPERCLKEN0 = CMU_HFPERCLKEN0_GPIO | BOOTLOADER_USART_CLOCKEN | AUTOBAUD_TIMER_CLOCK;
    CMU->HFPERCLKDIV = 1 << 8;

    // Enable LE and DMA interfaces.
    CMU->HFCORECLKEN0 = CMU_HFCORECLKEN0_LE | CMU_HFCORECLKEN0_DMA;

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
    // Mux PA0 (Green LED)
    GPIO->P[0].MODEL &= ~_GPIO_P_MODEL_MODE0_MASK;
    GPIO->P[0].MODEL |= GPIO_P_MODEL_MODE0_WIREDAND;
    GPIO->P[0].DOUTSET = (1 << 0);

    // Mux PB7 (Red LED)
    GPIO->P[1].MODEL &= ~_GPIO_P_MODEL_MODE7_MASK;
    GPIO->P[1].MODEL |= GPIO_P_MODEL_MODE7_WIREDAND;
    GPIO->P[1].DOUTCLR = (1 << 7);

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

static int test_boot_token(void)
{
    // If we find a valid boot token in RAM, the application is asking us explicitly
    // to enter DFU mode. This is used to implement the DFU_DETACH command when the app
    // is running.

    return boot_token.magic == BOOTLOADER_ENTER_TOKEN;
}

static int should_enter_bootloader(void)
{
    extern uint32_t __ram_start__;
    extern uint32_t __ram_end__;
    extern uint32_t __app_start__;
    extern uint32_t __app_end__;

    // Reset the boot token if we've just been powered up for the first time
    if (RMU->RSTCAUSE & RMU_RSTCAUSE_PORST)
    {
        boot_token.magic = 0;
        boot_token.boot_count = 0;
        boot_token.board_model = 0x23;
    }
    // Reset the "RSTCAUSE" value (EFM32HG-RM 9.3.1)
    RMU->CMD |= RMU_CMD_RCCLR;
    EMU->AUXCTRL |= EMU_CTRL_EMVREG;
    EMU->AUXCTRL &= ~EMU_CTRL_EMVREG;

    // If the special magic number is present, enter the bootloader
    if (test_boot_token())
    {
        bootloader_reason = BOOT_TOKEN_PRESENT;
        return 1;
    }

// If the user is holding the button down
#pragma warn "Check for button press here"
    if (0)
    {
        bootloader_reason = BUTTON_HELD_DOWN;
        return 1;
    }

    // If we've failed to boot many times, enter the bootloader
    if (boot_token.boot_count >= 3)
    {
        bootloader_reason = BOOT_FAILED_TOO_MANY_TIMES;
        return 1;
    }

    // Otherwise, if the application appears valid (i.e. stack is in a sane
    // place, and the program counter is in flash,) boot to it
    if (((appVectors[0] >= (uint32_t)&__ram_start__) && (appVectors[0] <= (uint32_t)&__ram_end__)) && ((appVectors[1] >= (uint32_t)&__app_start__) && (appVectors[1] <= (uint32_t)&__app_end__)))
    {
        bootloader_reason = NOT_ENTERING_BOOTLOADER;
        return 0;
    }

    // If there is no valid program, enter the bootloader
    bootloader_reason = NO_PROGRAM_PRESENT;
    return 1;
}

__attribute__((noreturn)) static void boot_app(void)
{
    // Relocate IVT to application flash
    __disable_irq();
    NVIC->ICER[0] = 0xFFFFFFFF;
    SCB->VTOR = (uint32_t)&appVectors[0];

    // Disable USB
    USB->GAHBCFG = _USB_GAHBCFG_RESETVALUE;
    USB->GINTMSK = _USB_GINTMSK_RESETVALUE;
    USB->DAINTMSK = _USB_DAINTMSK_RESETVALUE;
    USB->DOEPMSK = _USB_DOEPMSK_RESETVALUE;
    USB->DIEPMSK = _USB_DIEPMSK_RESETVALUE;
    USB->CTRL = _USB_CTRL_RESETVALUE;

    // Reset RTC settings.
    RTC->IEN = _RTC_IEN_RESETVALUE;
    RTC->COMP0 = _RTC_COMP0_RESETVALUE;
    RTC->CTRL = _RTC_CTRL_RESETVALUE;

    // Wait for LF peripheral syncronization.
    while (RTC->SYNCBUSY & _RTC_SYNCBUSY_MASK)
        ;
    while (CMU->SYNCBUSY & CMU_SYNCBUSY_LFACLKEN0)
        ;
    // Switch to default cpu clock.
    CMU->CMD = CMU_CMD_HFCLKSEL_HFRCO;
    CMU->OSCENCMD = CMU_OSCENCMD_HFXODIS
                  | CMU_OSCENCMD_AUXHFRCODIS
                  | CMU_OSCENCMD_LFRCODIS
                  | CMU_OSCENCMD_USHFRCODIS;
    CMU->USBCRCTRL = _CMU_USBCRCTRL_RESETVALUE;

    // Reset clock registers used
//    CMU->HFCORECLKEN0 = _CMU_HFCORECLKEN0_RESETVALUE;
    CMU->HFPERCLKDIV = _CMU_HFPERCLKDIV_RESETVALUE;
    CMU->HFPERCLKEN0 = _CMU_HFPERCLKEN0_RESETVALUE;
    CMU->LFCLKSEL = _CMU_LFCLKSEL_RESETVALUE;
    CMU->LFACLKEN0 = _CMU_LFACLKEN0_RESETVALUE;

    GPIO->P[0].MODEL = _GPIO_P_MODEL_RESETVALUE;
    GPIO->P[1].MODEL = _GPIO_P_MODEL_RESETVALUE;

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
          "r"(appVectors[0]),
          "r"(appVectors[1]));
    while (1)
        ;
}

__attribute__((noreturn)) void bootloader_main(void)
{
    if (should_enter_bootloader())
    {
        boot_token.magic = 0;
        boot_token.boot_count = 0;
        updater();
    }

    boot_app();
}