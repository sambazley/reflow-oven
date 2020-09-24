MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 6K
}

SECTIONS
{
    .text :
    {
        . = ALIGN(4);
        KEEP(*(vectors))
        . = ALIGN(4);
        *(.text*)
        KEEP(*(.init))
        KEEP(*(.fini))
        KEEP(*(.eh_frame))
        . = ALIGN(4);
    } > rom

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata*)
    } > rom

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > rom
    .ARM :
    {
        *(.ARM.exidx*)
    } > rom

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;

        *(.data*)

        . = ALIGN(4);
        _edata = .;
    } >ram AT >rom

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >ram

    end = .;

    .dynamic_allocations :
    {
        . = ALIGN(4);
        _ssystem_ram = .;
        . = . + 1000;
        . = ALIGN(4);
        _esystem_ram = .;
    } > ram

    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}
