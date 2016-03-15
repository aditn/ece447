        # Advanced branch test
	.text

        # J, JR, JAL, JALR, BEQ, BNE, BLEZ, BGTZ, BLTZ, BGEZ, BLTZAL, BGEZAL
        # BLTZAL, BGEZAL
main:
        addiu $v0, $zero, 0xa

        # Set up some comparison values in registers
        addiu $3, $zero, 1
        addiu $4, $zero, -1

        # Checksum register
        addiu $5, $zero, 0x1234

        # Test jump
        j l_1
l_0:
        #addiu $5, $zero, 1
        addu $5, $5, $ra
        beq   $zero, $zero, l_2
l_1:
        #addiu $6, $zero, 2
        addiu $5, $5, 7
        jal l_0
        j l_8
l_2:    
        #addiu $7, $zero, 3
        addiu $5, $5, 9
        bne $3, $4, l_4
l_3:
        #addiu $8, $zero, 4
        # Taken
        addiu $5, $5, 5
        bgez $zero, l_6
l_4:
        #addiu $9, $zero, 5
        # Not taken
        addiu $5, $5, 11
        blez  $3, l_3
l_5:
        #addiu $10, $zero, 6
        # Taken
        addiu $5, $5, 99
        bgtz  $3, l_3
l_6:
        #addiu $11, $zero, 7
        # here
        addiu $5, $5, 111
        jr $ra
        # Should go to l_1, then go to l_8
l_7:
        #addiu $12, $zero, 8
        # Should not get here
        addiu $5, $5, 200
        
        syscall
l_8:    
        #addiu $13, $zero, 9
        addiu $5, $5, 215
        jal l_10
l_9:
        #addiu $14, $zero, 10
        # Should not get here
        addiu $5, $5, 1
        syscall        
l_10:    
        #addiu $15, $zero, 11
        addu $5, $5, $6
        bltzal $4, l_12
l_11:
        #addiu $16, $zero, 12
        # Should not get here
        addiu $5, $5, 400
        syscall
l_12:    
        #addiu $17, $zero, 13
        addu $5, $5, $6
        bgezal $4, l_11
        
l_13:    
        #addiu $18, $zero, 14
        addiu $5, $5, 0xbeb0063d
        syscall
        
        
