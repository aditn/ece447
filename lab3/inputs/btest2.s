 # Advanced branch test
        .text

        # J, JR, JAL, JALR, BEQ, BNE, BLEZ, BGTZ, BLTZ, BGEZ, BLTZAL, BGEZAL
        # BLTZAL, BGEZAL
main:
        addiu $v0, $zero, 0xa

        # Set up some comparison values in registers
        addiu $3, $zero, 1
        addiu $4, $zero, -1

        # Checksum register
        #addiu $5, $zero, 0x1234

        # Test jump
        j l_1
l_0:
        addiu $5, $zero, 1
        #addu $5, $5, $ra
        beq   $zero, $zero, l_2
l_1:
        addiu $6, $zero, 2
        #addiu $5, $5, 7
        #jal l_0
        #j l_8
l_2:
        addiu $7, $zero, 3
        #addiu $5, $5, 9
        syscall
