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

We spent approximately 2 hours planning the design, 10 hours capturing the 
design, 3 hours testing the design, and 3 hours debugging.
