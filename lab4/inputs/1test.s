	.text
main:   
		nop
		addiu   $10, $zero, 1
        sub     $15, $zero, $10
        addiu   $v0, $zero, 0xa
        syscall
