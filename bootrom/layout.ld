SECTIONS
{
  . = 0;

  .text :
  {
    *(.start)
    *(.text*)
    *(.*data*)
    *(.bss)
  }
}
