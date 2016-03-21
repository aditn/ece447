.text
lui   $2, 0x1000        # set r2 at start of data seg
lui   $3, 0x5040	# set r3 to special pattern
ori   $3, $3, 0x3020 
addiu $4, $0, 1#10000	# set r4 to word count N	

loading:		# set first N word of data seg to pattern
sw    $3, 0($2)	
addiu $2, $2, 4
addiu $4, $4, -1
bne   $4, $0, loading

addiu $2, $0, 4#40000	# set r2 to byte count N*4	
lui   $3, 0x1000	# set r3 to start of data seg (src buf)
ori   $4, $3, 4#40000	# set r4 to data seg + N*4 (dest buf)

CopyLoop:		# copy byte by byte
lb    $5, 0($3)
sb    $5, 0($4)
addiu $3, $3, 1
addiu $4, $4, 1
addiu $2, $2, -1
bne   $2, $0, CopyLoop

addiu $2, $0, 1#10000	# set r2 to word count N
lui   $3, 0x5040
ori   $3, $3, 0x3020	# set r3 to special pattern
lui   $4, 0x1000	# set r4 to dest buf (data seg + N*4)
ori   $4, $4,4#40000 

CheckLoop:		# check dest buff
lw    $5, 0($4)
bne   $5, $3, Bad
addiu $4, $4, 4
addiu $2, $2, -1
bne   $2, $0, CheckLoop 
j     Finish     

Bad:
addiu $10, 0xbaadbaad

Finish:
addiu $2, $0, 10
syscall
