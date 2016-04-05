		.text

__start:        addiu $t0, $zero, 10
                nop
		addiu $t1, $t0, 0x1
                nop
		addiu $t2, $t1, 0x2
                syscall
