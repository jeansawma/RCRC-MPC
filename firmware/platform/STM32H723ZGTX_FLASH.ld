/* STM32H723ZGT6: 1 MiB Flash, 320 KiB AXI SRAM and 128 KiB DTCM.
 * DMA buffers and all .data/.bss are AXI-accessible; D-cache stays disabled.
 * Main/interrupt stack uses dedicated DTCM. */
ENTRY(Reset_Handler)
MEMORY
{
 FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
 RAM (xrw) : ORIGIN = 0x24000000, LENGTH = 320K
 DTCM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}
_estack = ORIGIN(DTCM) + LENGTH(DTCM);
_Min_Heap_Size = 0x1000;
_Min_Stack_Size = 0x8000;
SECTIONS
{
 .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
 .text : { . = ALIGN(4); *(.text .text*) *(.glue_7) *(.glue_7t) *(.eh_frame) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); _etext = .; } > FLASH
 .rodata : { . = ALIGN(4); *(.rodata .rodata*) . = ALIGN(4); } > FLASH
 .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
 .preinit_array (READONLY) : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } > FLASH
 .init_array (READONLY) : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } > FLASH
 .fini_array (READONLY) : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } > FLASH
 _sidata = LOADADDR(.data);
 .data : { . = ALIGN(4); _sdata = .; *(.data .data*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH
 .bss (NOLOAD) : { . = ALIGN(32); _sbss = .; __bss_start__ = .; *(.bss .bss*) *(COMMON) . = ALIGN(32); _ebss = .; __bss_end__ = .; } > RAM
 ._user_heap (NOLOAD) : { . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .); . += _Min_Heap_Size; } > RAM
 .stack (NOLOAD) : { . = ALIGN(8); . += _Min_Stack_Size; } > DTCM
 /DISCARD/ : { *(.note.GNU-stack) }
 .ARM.attributes 0 : { *(.ARM.attributes) }
}
ASSERT(_ebss + _Min_Heap_Size <= ORIGIN(RAM)+LENGTH(RAM), "AXI SRAM overflow")
