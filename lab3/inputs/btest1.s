        .text

main:
        addiu $v0, $zero, 0xa
l_0:
        ori $0, $0, 0
        ori $0, $0, 0
        addiu $5, $zero, 1
        #j l_1
        beq $zero, $zero, l_1
        ori $0, $0, 0
        ori $0, $0, 0
        addiu $10, $10, 0xb
        addiu $11, $zero, 0xc
        addiu $12, $zero, 0xd
l_1:	addiu $13, $zero, 0xe
        syscall
