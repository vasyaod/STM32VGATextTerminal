OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")

MEMORY
{
    rom (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00040000
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x0000c000
}

_eram = 0x20000000 + 0x0000c000;
end = _eram - 0x101;

/* Section Definitions */ 
SECTIONS 
{ 
    .text : 
    { 
        KEEP(*(.isr_vector .isr_vector.*))
        *(.text .text.*)
        *(.glue_7t) *(.glue_7)
        *(.rodata .rodata*)
    } > rom
    
    . = ALIGN(4); 
    _etext = .;
    _sidata = .; 
    		
    .data : AT (_etext) 
    { 
        _sdata = .; 
        *(.data .data.*) 
        . = ALIGN(4); 
        _edata = . ;
    } > ram

    /* .bss section which is used for uninitialized data */ 
    .bss (NOLOAD) : 
    { 
        _sbss = . ; 
        *(.bss .bss.*) 
        *(COMMON) 
        . = ALIGN(4); 
        _ebss = . ; 
    } > ram
        
    /* stack section */
    .co_stack (NOLOAD):             /* AT(_eram - 0x100) */
    {
        *(.co_stack .co_stack.*)
    } > ram

    . = ALIGN(4); 
    _end = .;
    __end__ = .;
} 