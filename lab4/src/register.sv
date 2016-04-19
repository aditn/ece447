
// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable)
       q <= d;

endmodule // register

//// clearRegister: A register which can be cleared
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module clearRegister(q, d, clk, enable, clr, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, clr, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (~clr)
       q <= reset_value;
     else if (enable)
       q <= d;
     /*else if (~enable)
       q <= reset_value;*/

endmodule // register


//// register2Input: A register which may be reset to one of two arbitrary values
////
//// q      (output) - Current value of register
//// d1     (input) - Possible next value of register
//// d2     (input) - Possible next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register2Input(q, d1,d2, clk, enable, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d1,d2;
   input [1:0]          enable;
   input                clk,rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (enable == 2'b10)
       q <= d1;
     else if (enable == 2'b01)
       q <= d2;

endmodule // register


//// cntlRegister: A register for propagating control signals.
////
//// All outputs are values of control signals at current stage.
//// All inputs (excluding clk, enable, and reset) are values of control signals at previous stage.
//// clk (input)     - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module cntlRegister (
   output logic ctrl_we, ctrl_Sys, ctrl_RI, regdst, jLink_en, alusrc1, alusrc2, se, hi_en, lo_en,
   output logic [1:0] memtoreg, pcMuxSel,
   output logic [3:0] alu__sel, mem_write_en,
   output logic [2:0] load_sel,brcond,
   output logic [1:0] store_sel,
   output logic       load_stall,
   input logic ctrl_we_in, ctrl_Sys_in, ctrl_RI_in, regdst_in, jLink_en_in, alusrc1_in, alusrc2_in, se_in, hi_en_in, lo_en_in,
   input logic [1:0] memtoreg_in, pcMuxSel_in,
   input logic [3:0] alu__sel_in, mem_write_en_in,
   input logic [2:0] load_sel_in, brcond_in,
   input logic [1:0] store_sel_in,
   input logic       load_stall_in,
   input logic clk, enable, rst_b);

   always_ff @(posedge clk or negedge rst_b)
     if (~rst_b)
       begin
         ctrl_we <= 1'b0;
         ctrl_Sys <= 1'b0;
         ctrl_RI <= 1'b0;
         regdst <= 1'b0;
         jLink_en <= 1'b0;
         alusrc1 <= 1'b0;
         alusrc2 <= 1'b0; 
         se <= 1'b0;
         hi_en <= 1'b0;
         lo_en <= 1'b0;
         memtoreg <= 2'b0; 
         pcMuxSel <= 2'b0;
         alu__sel <= 4'b0; 
         mem_write_en <= 4'b0;
         load_sel <= 3'b0;
         brcond <= 3'b0;
         store_sel <= 2'b0;
         load_stall <= 1'b0;
       end
     else if (enable)
       begin
         ctrl_we <= ctrl_we_in; 
         ctrl_Sys <= ctrl_Sys_in; 
         ctrl_RI <= ctrl_RI_in; 
         regdst <= regdst_in;
         jLink_en <= jLink_en_in;
         alusrc1 <= alusrc1_in; 
         alusrc2 <= alusrc2_in; 
         se <= se_in; 
         hi_en <= hi_en_in;
         lo_en <= lo_en_in;
         memtoreg <= memtoreg_in; 
         pcMuxSel <= pcMuxSel_in;
         alu__sel <= alu__sel_in;
         mem_write_en <= mem_write_en_in;      
         load_sel <= load_sel_in;
         brcond <= brcond_in;
         store_sel <= store_sel_in;
         load_stall <= load_stall_in;
       end
     else if (~enable)
       begin
         ctrl_we <= 1'b0;
         ctrl_Sys <= ctrl_Sys_in;
         ctrl_RI <= ctrl_RI_in;
         regdst <= regdst_in;
         jLink_en <= jLink_en_in;
         alusrc1 <= alusrc1_in;
         alusrc2 <= alusrc2_in;
         se <= se_in;
         hi_en <= 1'b0;
         lo_en <= 1'b0;
         memtoreg <= memtoreg_in;
         pcMuxSel <= pcMuxSel_in;
         alu__sel <= alu__sel_in;
         mem_write_en <= 1'b0;
         load_sel <= `NO_LOAD;
         brcond <= brcond_in;
         store_sel <= store_sel_in;
         load_stall <=load_stall_in;
       end

endmodule

////
//// countdownReg: keeps track of distance when hazard detected
////
//// CDen (input)    - enable bit for countdown
//// CDAmt (input)   - distance to countdown by
//// clk (input)     - Clock (positive edge-sensitive)
//// reset  (input)  - System reset
//// stall (output) - signal that an instruction is currently being stalled
////
module countdownReg #(parameter reset_value = 0) (
  input logic CDen, clk, rst_b,
  input logic [2:0] CDAmt,
  output logic stall);
  
  logic [2:0] CDAmtq; 
  assign stall = (CDAmtq == 3'd0) ? 1'b0: 1'b1;

  always_ff @(posedge clk or negedge rst_b) begin
    if (~rst_b) begin
      CDAmtq <= 3'b0;
    end
    else if (CDen) begin
      CDAmtq <= CDAmt;
    end
    else if (CDAmtq != 3'd0) begin
      CDAmtq <= CDAmtq-1;
    end
  end

endmodule
