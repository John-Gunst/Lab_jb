    # RISCV.asm
    .section .text
    .globl _start

_start:

    # Read switches and scale
    csrrw   t0, 12'hf00, x0     
    # t0 := CSR[0xF00] (switches)
    call    mul_by_100k_shr14   # returns scaled fixed-point in t0


    # Binary-search sqrt (fixed-point)
    li      t1, 0               # low = 0
    li      t2, 8388608         # high = 2^23 [512.0 in 8.18 fixed point]

loop:
    bge     t1, t2, done        # while low < high
    add     t3, t1, t2
    srli    t3, t3, 1           # mid = (low + high) >> 1

    # compute mid^2 >> 14 using mul64
    mv      a0, t3
    mv      a1, t3
    call    mul64               # a0 = lo, a1 = hi (64-bit product)
    srli    t4, a0, 14          # lo >> 14
    slli    t5, a1, 18          # hi << 18
    or      t4, t5, t4          # t4 = (mid^2) >> 14

    # compare unsigned, so if mid^2_scaled > scaled_input then high = mid, else low = mid + 1
    bgtu    t4, t0, high_label
    addi    t1, t3, 1
    j       loop

high_label:
    mv      t2, t3
    j       loop

done:
    addi    t1, t1, -1          # result = low - 1


    # Convert result to display word using bin2dec

    mv      a0, t1              # pass fixed point result in a0
    call    bin2dec             # returns packed 32 bit display word in a0

    # Write to HEX CSR io2
    csrrw   x0, 12'hf02, a0

# Hang forever
halt:
    j halt


# mul64: 32x32 > 64 unsigned multiply

mul64:
    mv      t0, a0      # t0 = multiplicand
    mv      t1, a1      # t1 = multiplier
    li      t2, 0       # acc_lo
    li      t3, 0       # acc_hi

mul64_loop:
    beqz    t1, mul64_done
    andi    t4, t1, 1
    beqz    t4, mul64_skip_add
    # acc += t0  (64-bit add)
    add     t5, t2, t0
    sltu    t6, t5, t2        # carry
    mv      t2, t5
    add     t3, t3, t6

mul64_skip_add:
    slli    t0, t0, 1
    srli    t1, t1, 1
    j       mul64_loop

mul64_done:
    mv      a0, t2      # low
    mv      a1, t3      # high
    ret


# mul_by_100k_shr14:
mul_by_100k_shr14:
    mv      a0, t0           # multiplicand
    li      a1, 100000       # multiplier
    call    mul64            # returns a0=lo, a1=hi

    # compute (hi:lo) >> 14  => (hi << 18) | (lo >> 14)
    srli    t1, a0, 14
    slli    t2, a1, 18
    or      t0, t2, t1
    ret


# div10: unsigned divide by 10 (software)
div10:
    mv      t0, a0          # rem := dividend
    li      t1, 0           # q := 0
    li      t4, 10          # val := 10
    li      t5, 1           # q_bit := 1

div10_scale:
    slli    t3, t4, 1
    bgtu    t3, t0, div10_scaled
    slli    t4, t4, 1
    slli    t5, t5, 1
    j       div10_scale

div10_scaled:
div10_loop:
    blt     t0, t4, div10_nosub
    sub     t0, t0, t4
    or      t1, t1, t5
div10_nosub:
    srli    t4, t4, 1
    srli    t5, t5, 1
    bnez    t5, div10_loop

    mv      a0, t1       # quotient
    mv      t2, t0       # remainder
    ret


# bin2dec: convert unsigned 32-bit integer in a0 to 8-digit decimal packed
bin2dec:
    mv      t0, a0       # working copy
    li      t11, 0       # index

    # extract 8 digits (LSB first). Remainders go into t3..t10
    li      t12, 0
bin2dec_extract_loop:
    mv      a0, t0
    call    div10        # returns quotient in a0, remainder in t2
    mv      t0, a0       # update working value

    # store remainder in appropriate register t3..t10
    beqz    t12, store_d0
    li      t13, 1
    beq     t12, t13, store_d1
    li      t13, 2
    beq     t12, t13, store_d2
    li      t13, 3
    beq     t12, t13, store_d3
    li      t13, 4
    beq     t12, t13, store_d4
    li      t13, 5
    beq     t12, t13, store_d5
    li      t13, 6
    beq     t12, t13, store_d6
    # else index 7
    mv      t10, t2
    j       bin2dec_next

store_d0:
    mv      t3, t2
    j       bin2dec_next
store_d1:
    mv      t4, t2
    j       bin2dec_next
store_d2:
    mv      t5, t2
    j       bin2dec_next
store_d3:
    mv      t6, t2
    j       bin2dec_next
store_d4:
    mv      t7, t2
    j       bin2dec_next
store_d5:
    mv      t8, t2
    j       bin2dec_next
store_d6:
    mv      t9, t2
    j       bin2dec_next

bin2dec_next:
    addi    t12, t12, 1
    li      t13, 8
    blt     t12, t13, bin2dec_extract_loop

    # Pack digits into a0: t10 (MSB) ... t3 (LSB)
    li      a0, 0
    slli    a0, a0, 4
    or      a0, a0, t10
    slli    a0, a0, 4
    or      a0, a0, t9
    slli    a0, a0, 4
    or      a0, a0, t8
    slli    a0, a0, 4
    or      a0, a0, t7
    slli    a0, a0, 4
    or      a0, a0, t6
    slli    a0, a0, 4
    or      a0, a0, t5
    slli    a0, a0, 4
    or      a0, a0, t4
    slli    a0, a0, 4
    or      a0, a0, t3

    ret