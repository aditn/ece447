////
//// flushMod: module for flushing if branch
////
//// pcMuxSelFinal (input) - determines if there is a branch
//// pc_ID (input) - address of instruction in ID stage
//// pc_EX (input) - address of instruction in EX stage
//// br_target (input) - branch target calculated in EX stage
//// rs_fwd (input) - JR target
//// j_target (input) - jump target 
//// flush (input) - signal that indicates an instruction is currently being stalled
//// EXen (output) - register enable bits at the EX stage
//// CDFlushen  (output) - enables countdown register
//// mispredict (output) - signals when a misprediction is detected
//// CDAmt (output) - number of clock cycles to stall
////
module flushMod(
  input logic [1:0] pcMuxSelFinal, pcMuxSel_EX,
  input logic [31:0] pc_ID, pc_EX, br_target, rs_fwd, j_target,
  input logic IDswap_EX, flush,
  output logic EXen_1, EXen_2, MEMen_2, CDFlushen, mispredict,
  output logic [2:0] CDFlushAmt);

  always_comb begin
    EXen_1 = 1'b1;
    EXen_2 = 1'b1;
    MEMen_2 = 1'b1;
    CDFlushAmt = 3'b0;
    CDFlushen = 1'b0;
    mispredict = 1'b0;
    if (flush == 1'b1) begin
      EXen_1 = 1'b0;
      EXen_2 = 1'b0;
      MEMen_2 = 1'b0;
    end
    else if(pcMuxSel_EX!=2'b00 && flush==1'b0 && ((pcMuxSelFinal==2'b00 && IDswap_EX==1'b1 && pc_ID!=pc_EX+4) ||
                                                  (pcMuxSelFinal==2'b00 && IDswap_EX==1'b0 && pc_ID!=pc_EX+8) ||
                                                  (pcMuxSelFinal==2'b01 && pc_ID!=br_target) ||
                                                  (pcMuxSelFinal==2'b10 && pc_ID!=rs_fwd) ||
                                                  (pcMuxSelFinal==2'b11 && pc_ID!=j_target))) begin
      CDFlushen = 1'b1;
      EXen_1 = 1'b0;
      EXen_2 = 1'b0;
      MEMen_2 = 1'b0;
      CDFlushAmt = 3'd1;
      mispredict = 1'b1;
    end
  end
endmodule
