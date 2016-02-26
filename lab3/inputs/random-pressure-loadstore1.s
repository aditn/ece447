.text
 lui $a0, 0x1000
 li $t0, 0x12345678
 sw $t0, 0($a0)
 li $t0, 0
 li $t1, 0
 li $t2, 0
 li $t3, 0
 li $t4, 0
 li $t5, 0
 li $t6, 0
 li $t7, 0
 and   $t0 , $t0 , $t2 
 addiu   $t1 , $t2 , 924217013 
 xori   $t2 , $t0 , 3908471170 
 lhu   $t1 , 0($a0) 
 sw   $t0 , 0($a0) 
 lw   $t2 , 0($a0) 
 or   $t1 , $t1 , $t1 
 lhu   $t2 , 0($a0) 
 and   $t1 , $t2 , $t1 
 lhu   $t2 , 0($a0) 
 xori   $t2 , $t0 , 3166070588 
 addu   $t0 , $t1 , $t0 
 addu   $t2 , $t2 , $t2 
 xor   $t2 , $t1 , $t2 
 addiu   $t2 , $t1 , 2173033926 
 lh   $t0 , 0($a0) 
 sh   $t1 , 0($a0) 
 addu   $t1 , $t0 , $t0 
 lbu   $t2 , 0($a0) 
 ori   $t1 , $t0 , 3117249512 
 addiu   $t2 , $t2 , 1052882209 
 lw   $t1 , 0($a0) 
 lhu   $t0 , 0($a0) 
 sw   $t0 , 0($a0) 
 or   $t0 , $t0 , $t0 
 addiu $v0, $0, 10
 syscall

