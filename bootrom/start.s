.section .start
reset:
    j _start
    j .
    j .
    j .
    j .

_start:
    li t0, 0x100
.delay:
    addi t0, t0, -1
    bnez t0, .delay

    call main

    li t0, 0x42
    li t1, 0x01000000
    sb t0, (t1)
    j .
