.section .start
reset:
    j _start
    j .
    j .
    j .

_start:
    call main

    li t0, 0x42
    li t1, 0x01000000
    sb t0, (t1)
    j .
