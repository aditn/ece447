        # Basic arithmetic instructions
        # This is a hodgepodge of arithmetic instructions to test
        # your basic functionality.
        # No overflow exceptions should occur
	.text
main:   
        addiu   $2, $zero, 1024
        add     $5, $zero, 1234
        sll     $6, $5, 16
        addiu   $7, $6, 9999
        subu    $8, $7, $2
        sub     $15, $zero, $10
        lui     $17, 100
        addiu   $v0, $zero, 0xa
        syscall
        
        
                        
