            .text

__start:    addi $3, $zero, 1
            addiu $2, $zero, 0xa
			mthi $3
			mfhi $4
			addiu $4, $4, 10
			mtlo $3
			mflo $5
			addiu $4, $4, 10
			addi $6, $5 -2
            syscall

