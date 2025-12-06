# older sqrt and bcd conversion framos
.text
.globl _start

_start:
    
    csrrw   a0, 0xf00, x0
    #li a0,2
    slli    a0, a0, 14

    addi    sp, x0, 0
    addi    gp, x0, 256
    slli    gp, gp, 14

sqrt_loop:

    mul     tp, sp, sp
    mulhu   t1, sp, sp
    srli    tp, tp, 14 #upper frac
    slli    t1, t1, 18 #lower frac
    or      tp, tp, t1
    beq     tp, a0, sqrt_done
    bltu    tp, a0, sqrt_too_small

sqrt_too_large:

    sub     sp, sp, gp             
    j       sqrt_continue

    

sqrt_too_small:

    add     sp, sp, gp             

    
sqrt_continue:

    srli    gp, gp, 1              
    bnez    gp, sqrt_loop          

    

sqrt_done:
    add     x1, x0, sp
    li      a3, 100000 
    mul     x2, x1, a3
    mulhu   x1, x1, a3
    slli    x1, x1, 18
    srli    x2, x2, 14
    or      x1, x1, x2
    add     t0, x0, x1



    lui     t1, 0x1999a
    addi    t1, t1, -0x666
    addi    t2, x0, 10
    addi    s0, x0, 0
    addi    t6, x0, 8
    addi    a2, x0, 0

bcd_loop:

    mulhu   t4, t0, t1
    mul     t5, t4, t2
    sub     t5, t0, t5
    andi    t5, t5, 0xF
    sll     t5, t5, a2
    or      s0, s0, t5



    addi    t0, t4, 0
    addi    a2, a2, 4
    addi    t6, t6, -1
    bnez    t6, bcd_loop
    csrrw   x0, 0xf02, s0
