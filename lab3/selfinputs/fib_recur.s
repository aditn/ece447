        la  $sp, stack_start
	addi $a0, $zero, 0x004	# n
	jal fibr
        addiu $v1, $v0, 0 
	addiu $v0, $zero, 10
	syscall

	# Example from P&H 
fibr:
	addi $sp, $sp, -12  	# adjust stack for 3 items
	sw   $ra, 4($sp)      	# save return address
	sw   $a0, 0($sp)      	# save argument
	slti $t0, $a0, 2      	# test for n < 2
	beq  $t0, $zero, L1

	add  $v0, $zero, $a0    # if so, result is n
	addi $sp, $sp, 12      	#   pop 3 items from stack
	jr   $ra              	#   and return


L1:	addi $a0, $a0, -1     	# arg=n-1
	jal  fibr             	# recursive call
	sw   $v0, 8($sp)	# save fib(n-1) to stack

	lw   $a0, 0($sp)       	# restore original n
	addi $a0, $a0, -2	# arg=n-2
	jal  fibr             	# recursive call

	lw   $a0, 8($sp)	# recall fib(n-1)
	add  $v0, $v0, $a0	# sum fib(n-1) and fib(n-1)

	lw   $ra, 4($sp)      	# restore return address
	addi $sp, $sp, 12      	# pop 3 items from stack
	jr   $ra              	# and return


	# KLUDGE ALERT
	# putting stack in the text segment to work with the class simulator
	# size 200 stack, keep n under 60; don't bother to try though.
stack_bottom:
	.word 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	.word 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
	
stack_start:
	.word 0xdeadbeef
	
	


	
