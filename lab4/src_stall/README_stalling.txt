In our design, we added clusters of registers in order to separate data values
between each of the five stages. The registers propagate the necessary data 
used in each of the stages in the pipeline such as control signals and 
register data. To facilitate stalling, we implemented a stall detector to
detect hazards and activate a  countdown register which stalls for the 
appropriate number of clock cycles until there is no longer a hazard.

Hazard Distance Analysis:
ALU R/I then any instruction: RAW hazard distance = 3
LW then any instruction: RAW hazard distance = 3
MF then any instruction (except MT): RAW hazard distance = 3
No other hazards

The critical path of our synthesized is through the decoder, the control bit 
register for execute stage, the mux to choose the first input for the ALU 
(either Rs data or Rt data), the ALU, and the register to hold the ALU output 
in the memory stage. The minimum clock cycle is 7.79 ns. We improved our clock
frequency by 0.8 ns.
Since our execute stage seems to be using the most time, we could possibly
divide the execute stage into two stages in order to improve our clock
frequency further.

We spent approximately 2 hours planning the design, 10 hours capturing the 
design, 3 hours testing the design, and 3 hours debugging.
