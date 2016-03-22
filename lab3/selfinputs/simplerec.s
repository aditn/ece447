        la  $sp, stack_start
        addi $a0, $zero, 0x001  # n
#add $t9, $sp, $zero
        jal sum
        addiu $v1, $v0, 0 
        addiu $v0, $zero, 10
        syscall

sum: 
        addi $sp, $sp, -8      # adjust stack for 3 items
        sw   $ra, 4($sp)        # save return address
        sw   $a0, 0($sp)        # save argument
        slti $t0, $a0, 2        # test for n < 2
        beq  $t0, $zero, base
        addi $a0, $a0, -1
        jal  sum
        lw   $a0, 0($sp)
        add  $v0, $v0, $a0
        j    ret

base:   add  $v0, $zero, $a0    # if so, result is n

ret:    lw   $ra, 4($sp)
        addi $sp, $sp, 8       #   pop 3 items from stack
        jr   $ra                #   and return




stack_bottom:
        .word 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
        .word 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0

stack_start:
        .word 0xdeadbeef
