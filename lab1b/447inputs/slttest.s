        .text
__start:    addiu $v0, $zero, 10
        addiu $4, $zero, 5
        slt $10, $zero, $4

        addiu $5, $zero, -1
        slt $11, $zero, $5
        slt $12, $5, $zero
        
        sltu $13, $zero, $5
        sltu $14, $5, $zero

        slti $15, $zero, -1
        slti $16, $zero, 1
        sltiu $17, $zero, -1
        sltiu $18, $zero, 0

        syscall
