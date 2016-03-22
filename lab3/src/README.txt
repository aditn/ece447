In this pipelined implementation, the design is split into the 5 stages
(IF,ID,EX,MEM,WB) by having registers at each stage to store necessary 
value. Most control bits were also propogated throughout the stages EX,
MEM, and WB. We kept out stalling checking/enabling module nearly intact,
however we only allowed a stall for RAW hazards on load instructions that
were at the time in the EX stage. The forwarding module was then added to
pass the calculated value from the beginning of the MEM stage or the
beginning of the WB stage to the EX stage for the instruction that is 
causing a RAW hazard.

The new component for this lab was incorporating control flow with the 
pipelined implementation. This works by predicting whether or not a
branch statement would be taken before the actual calculations were
done by the ALU. The prediction would take place in the IF stage while
the actual calculation would be done in the EX stage. The benefit of this
is that if the prediction is correct, then we save 2 stages worth of time.
If the prediction was incorrect, then the stages that were processing the
wrong instructions would need to be flushed from the pipeline. This included
adding a Branch Target Buffer (BTB) and error correction if the branch
prediction was incorrect. The error correction included a module to flush
the stages where the incorrect instructions were being processed (the current
ID and EX stages). We do this by flushing the EX stage over 2 clock cycles.
This flush module works in a very similar fashion to the stall detector that
was built to hand RAW hazards. The BTB works as stated in the lab handout.
In the IF stage, we index through the BTB based on the PC value, and find
a prediction based on it. The BTB is then updated after actual values are
calculated by the ALU in the EX stage.

Hazard Distance Analysis:
LW then any instruction: RAW hazard distance = 1
An instruction that writes to a register, and then JR reads from the register
directly after will cause a hazard: RAW hazard distance = 1

The critical path is through the PC_EX register, actual branch_target_EX
calculations, the flush module, the muxes that determine the new history
state to write back to the BTB, and the BTB.
The minimum clock cycle is 9.70 ns. The minimum clock cycles is of a similar
time to the previous lab where forwarding was incorporated.

We spent about 3 hrs on planning the design, 8 hrs capturing the design, 4
hr for testing the design, 25 hrs for debugging, and 2 hours for analyzing
performance.

