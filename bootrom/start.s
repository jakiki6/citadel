.section .start
_start:
    call main

    li t1, 0x01000000
    sw x0, (t1)
    j .
