This design is very similar to the stalling design in that we first split
the design into the 5 stages (IF,ID,EX,MEM,WB) by having registers at 
each stage to store necessary value. Most control bits were also propogated
throughout the stages EX, MEM, and WB. We kept out stalling checking/
enabling module nearly intact, however we only allowed a stall for RAW 
hazards on load instructions that were at the time in the EX stage. The 
forwarding module was then added to pass the calculated value from the
beginning of the MEM stage or the beginning of the WB stage to the EX
stage for the instruction that is causing a RAW hazard.

Hazard Distance Analysis:
LW then any instruction: RAW hazard distance = 1
No other hazards


The critical path is through the output of the ALU in the MEM stage, then
through the forwarding module, then through the ALU inputs, through the
ALU and ending at the output of the ALU in the MEM stage again.
The minimum clock cycle is 9.07 ns.

Comparing the clock cycles for stalling (7.79ns) and forwarding (9.07ns),
forwarding would need to improve our IPC by approximately 16.4% to break
even.

We spent about 1 hr on planning the design, 2 hr capturing the design, 1
hr for testing the design and 1 hr for debugging.
