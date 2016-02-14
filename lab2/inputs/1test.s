		.text

__start:        addiu $t0, $zero, 10
                addiu $t1, $t0, 0x1
                addiu $t2, $t1, 0x2
                syscall
