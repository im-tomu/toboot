/*
 * EFM32 memory setup.
 */

ENTRY(Reset_Handler)

MEMORY
{
    bl_flash   :  org = 0x00000000,  len = 8k,
    app_flash  :  org = 0x00004000,  len = 0xC000,
    ram        :  org = 0x20000008,  len = 8k - 8
}

__bl_start__            = ORIGIN(bl_flash);
__bl_size__             = LENGTH(bl_flash);
__bl_end__              = __bl_start__ + __bl_size__;
__app_start__           = ORIGIN(app_flash);
__app_end__             = __app_start__ + LENGTH(app_flash);
__ram_start__           = ORIGIN(ram);
__ram_size__            = LENGTH(ram);
__ram_end__             = __ram_start__ + __ram_size__;
boot_token = ORIGIN(ram) - 8;

SECTIONS
{
    .text : ALIGN(4) SUBALIGN(4) {
        . = 0;
        KEEP(*(.vectors))
        KEEP(*(.toboot_configuration))
        *(.startup*)
    } > bl_flash = 0xFF
    _eflash = .;

    .apptext (NOLOAD) : {
        . = ORIGIN(app_flash);
        *(.appvectors)
    } > app_flash = 0xFF

    /* Combined data and text, after relocation */
    .dtext : AT (_eflash) {
        . = ALIGN(4);
        _sdtext = .;
        *(.text*)
        *(.ramtext*)
        *(.rodata*)
        *(.data*)
        . = ALIGN(4);
        _edtext = .;
    } > ram

    .noinit (NOLOAD) : {
        *(.noinit*)
    } > ram

    .bss : {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end = .;
    } > ram

    __main_stack_end__ = ORIGIN(ram) + LENGTH(ram);

    /DISCARD/ : {
        *(.note.GNU-stack)
        *(.gnu_debuglink)
        *(.note.gnu.build-id)
        *(.note.gnu.build-id.*);
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
        *(.interp)
        *(.ARM.exidx*)
        *(.gnu.linkonce.armexidx.*)
    }

  /* ld.bfd will segfault without these */
  .rel.plt : {
    . = ALIGN(4);
  }
  .got.plt : {
    . = ALIGN(4);
  }
}

/* RAM region to be used for Main stack. This stack accommodates the processing
      of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram);

/* RAM region to be used for the process stack. This is the stack used by
      the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram);
