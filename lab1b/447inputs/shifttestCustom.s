        # Basic shift tests
        .text
main:   
        addiu $3, $zero, 4
        addiu $4, $zero, 0xf0
        srav  $5, $4, $3

        addiu $6, $zero, 0x80000000
        addiu $7, $zero, -1
        srav  $8, $6, $7
        sra   $9, $6, -1
        srav  $10, $6, $3
        sra   $11, $6, 4

        addiu $3, $zero, 0x100004
        srlv  $12, $6, $3
        sllv  $13, $7, $3
        srlv  $14, $7, $3

        addiu $2, $zero, 0xa
        syscall
