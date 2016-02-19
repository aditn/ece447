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

            addiu $t0, $zero, 0x5
            mtlo $t0
            addiu $t2, $t0, 0x1
            addiu $t1, $zero, 0x1
            mflo $t1
            addiu $t3, $t1, 0x1
            syscall

