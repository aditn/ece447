        .text

main:
        addiu $v0, $zero, 0xa
        addiu $3, $zero, 1
        addiu $4, $zero, -1
        j l_1
l_0:
        ori $0, $0, 0
        ori $0, $0, 0
        addiu $5, $zero, 1
        jal l_2
        #beq $zero, $zero, l_1
        #ori $0, $0, 0
        #ori $0, $0, 0
        addiu $10, $10, 0xb
        addiu $11, $zero, 0xc
        addiu $12, $zero, 0xd

l_1:	addiu $13, $zero, 0xe
        j l_0
        #should not get here
        addiu $13, $zero, 0x1
        syscall

l_2:    addiu $14, $zero 0xf
        syscall
