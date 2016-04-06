        # Basic LW/SW test
	.text
main:
        #;;  Set a base address
        lui    $3, 0x1000       #1

        addiu  $5, $zero, 255   #2
        add    $6, $5, $5       #1
        add    $7, $6, $6       #2
        addiu  $8, $7, 30000    #1
        
        #;; Place a test pattern in memory
        sw     $5, 0($3)        #2
        sw     $6, 4($3)        #1
        sw     $7, 8($3)        #2
        sw     $8, 12($3)       #1

        lw     $9,  0($3)       #2
        lw     $10, 4($3)       #1
        lw     $11, 8($3)       #2
        lw     $12, 12($3)      #1

        addiu  $3, $3, 4        #1
        sw     $5, 0($3)        #2
        sw     $6, 4($3)        #1
        sw     $7, 8($3)        #2
        sw     $8, 12($3)       #1

        lw     $13,  -4($3)     #2
        lw     $14,  0($3)      #1
        lw     $15,  4($3)      #2
        lw     $16,  8($3)      #1
               
        #;; Calculate a "checksum" for easy comparison
        add    $17, $zero, $9   #2
        add    $17, $17, $10    #1
        add    $17, $17, $11    #2
        add    $17, $17, $12    #1
        add    $17, $17, $13    #2
        add    $17, $17, $14    #1
        add    $17, $17, $15    #2
        add    $17, $17, $16    #1
        
        #;;  Quit out 
        addiu $v0, $zero, 0xa   #2
        syscall                 #1
        
