	.text
main:   
		addiu   $5, $zero, 1024
        addu    $3, $5, $5
        nop
        nop
        nop
        or      $4, $3, $5
        addiu   $v0, $zero, 0xa
        syscall
