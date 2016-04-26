In this superscalar pipelined implementation, the design is split into
2 instances of the 5 stages (IF,ID,EX,MEM,WB) by having registers at
each stage to store necessary value. The 2 pipelines are nearly identical
and the control bits/data values propogated in previous labs were now doubled so
each pipeline has its own set of control bits and data values. We kept
forwarding mostly intact. However, now if there is a RAW hazard between
an instruction in pipeline 2 and an instruction in pipeline 1, we forward
the value from pipeline 2 to pipeline 1. Also, if in this situation, the 
two instructions are in the same stage (both in ID), then stalling would
take place for the ID and IF stages in pipeline 1 and the IF stage in
pipeline 2 until the data value would be received. Another special situation
to account for is if there is a RAW hazard between pipeline 1 and 2 where
pipeline 1 is the primary pipeline. To deal with this situation efficiently,
we swap the instruction from pipeline 2 with the instruction inthe stage
before the conflicting instruction in pipeline 1. This prevents a stall from
occuring since forwarding would be used to resolve the hazard.


/***** NEED TO DO CONTROL FLOW*****/

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


Optimizations:
Our main optimization was writing our own adder. Since DC synthesizes to a RCA
(which is slow), we decided to write our own carry-select adder. This was used
in place of '+' throughout the code. The significant impact of this adder 
occured in the ALU. 
We also wrote our own arithmatic right shift which was used in place of the
'>>>' operator. According to sources online, this change would provide
significant benefits in the ALU in terms of clock cycle time. In the end,
these two changes allowed for a significant drop in clock cycle time.


Hazard Distance Analysis:
/***** NEED TO DO *****/


The critical path is through the mux for choosing the ALU inputs, the ALU,
the choose next PC mux which chooses between possible branch values,the
flush Module, and the register that stores in the PC value in the EX stage.
The minimum clock cycle is 7.07 ns, which is a decrease from 9.70 from the
lab 3 implementation.

We spent about 3 hrs on planning the design, 15 hrs capturing the design, 4
hr for testing the design, 12 hrs for debugging, and 2 hours for analyzing
performance.

