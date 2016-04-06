        # Basic arithmetic instructions
        # This is a hodgepodge of arithmetic instructions to test
        # your basic functionality.
        # No overflow exceptions should occur
	.text
main:   
        addiu   $2, $zero, 1024 #1
        addu    $3, $2, $2      #2
        nop                     #1
        nop                     #2
        or      $4, $3, $2      #1
        add     $5, $zero, 1234 #2
        sll     $6, $5, 16      #1
        addiu   $7, $6, 9999    #2
        subu    $8, $7, $2      #1
        xor     $9, $4, $3      #2
        xori    $10, $2, 255    #1
        srl     $11, $6, 5      #2
        sra     $12, $6, 4      #1
        and     $13, $11, $5    #2
        andi    $14, $4, 100    #1
        nop                     #2
        sub     $15, $zero, $10 #1
        lui     $17, 100        #2
        addiu   $v0, $zero, 0xa #1
        syscall                 #2
        
        
                        
