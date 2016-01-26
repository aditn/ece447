        # Basic addition tests
	.text
main:   
        addi $4, $zero, 2
        addi $5, $zero, 3
        addi $6, $zero, -1
        mthi $4
        mtlo $5
        mult $4, $5
        mfhi $20
        mflo $21
        div  $5, $4
        syscall
