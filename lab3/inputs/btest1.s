        .text

main:
        addiu $v0, $zero, 0xa
l_0:
        addiu $5, $zero, 1
        #j l_1
        bgtz  $5, l_1 
        addiu $10, $10, 0xb
        addiu $11, $zero, 0xc
        addiu $12, $zero, 0xd
l_1:	syscall
