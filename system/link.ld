/* Link file for STM32F765VGT6 */

MEMORY
{
  ROM (rx)       : ORIGIN = 0x08000000, LENGTH = 1M
  RAM (rwx)      : ORIGIN = 0x20020000, LENGTH = 384k
  RAM_DTCM (rwx) : ORIGIN = 0x20000000, LENGTH = 128k
  RAM_ITCM (rwx) : ORIGIN = 0x00000000, LENGTH = 16k
  RAM_BCKP (rwx) : ORIGIN = 0x40024000, LENGTH = 4k
}

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector : ALIGN(16) {
    KEEP(*(.isr_vector))
  } > ROM

  .preinit_array : ALIGN(4) {
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;
  } > ROM

  .init_array : ALIGN(4) {
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;
  } > ROM

  .fini_array : ALIGN(4) {
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;
  } > ROM

  .text ALIGN(16) : ALIGN(16) {
    *(.text)
    *(.text.*)
    *(.glue_7t)
    *(.glue_7)
    *(.gcc*)
  } > ROM

  .rodata : ALIGN(4) {
    __rodata_start = .;
    *(.rodata)
    *(.rodata.*)
    __rodata_end = .;
  } > ROM

  . = ALIGN(4);
  __text_end = .;

  .data : ALIGN(4) {
    __data_start = .;
    *(.data*)
    *(.data.*)
    . = ALIGN(4);
    __data_end = .;
  } > RAM AT > ROM

  . = ALIGN(4);

  .bss (NOLOAD) : ALIGN(4) {
    __bss_start = .;
    *(.bss*)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end = .;
  } > RAM

  . = ALIGN(4);
  __all_end = .;
}

PROVIDE(__stack = ORIGIN(RAM) + LENGTH(RAM));
