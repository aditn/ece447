            .text

__start:    addi $3, $zero, 1
			mthi $3
			nop
			mfhi $4
			addiu $t0, $zero, 10
            syscall

