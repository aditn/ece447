/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"
//`include "mips_alu.sv"

typedef struct packed{
   logic exception_halt, syscall_halt, internal_halt;
   logic load_epc, load_bva, load_bva_sel, load_ex_regs;
   logic [31:0]   epc, cause, bad_v_addr;
   logic [4:0]    cause_code;

   logic[1:0] pcMuxSel;
   logic[1:0] pcMuxSelFinal;

  logic [31:0]   inst;
  logic [31:0]   pc;
  logic [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
  logic [5:0]    dcd_op, dcd_funct2;
  logic [4:0]    dcd_rs, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
  logic [15:0]   dcd_offset, dcd_imm;
  logic [25:0]   dcd_target;
  logic [19:0]   dcd_code;
  logic          dcd_bczft;
  logic [31:0] inst_ID;
  
  logic [3:0]   alu__sel;   // From Decoder of mips_decode.v
  logic     ctrl_RI;    // From Decoder of mips_decode.v
  logic     ctrl_Sys;   // From Decoder of mips_decode.v
  logic     ctrl_we;    // From Decoder of mips_decode.v

  logic [31:0]   rt_data, rs_data, rd_data, alu__out;
  // End of automatics
  
  //Added control signals
  logic regdst;
  logic jLink_en;
  logic[2:0] brcond;
  logic branchTrue;
  logic [1:0] memtoreg;
  logic alusrc1;
  logic alusrc2;
  logic se;
  logic [3:0] mem_en; //memory write enable
  logic [2:0] load_sel; //selects the type of load for the loader to perform
  logic hi_en;
  logic lo_en;
  logic [1:0] store_sel; //selects the type of store for the storer to perform
  logic load_stall; //check if instruction is a load or MF to signal a stall
  logic stall;

  logic [4:0] wr_reg; //input to write register
  logic [31:0] imm; //signed or unsigned immediate
  logic [31:0] wr_data; //data to write to register file
  logic [31:0] wr_dataMem; //intermediate data to write to register file
  logic [4:0] wr_regNum;//intermediate reg to write to 
  logic [4:0] rt_regNum;//rt register to read from

  logic [31:0] br_target; //branch target
  logic [31:0] j_target; //unconditional jump target
  logic [31:0] hi_out; //HI Register out
  logic [31:0] lo_out; //LO Register out
  logic [31:0] hi_in; //HI Register in
  logic [31:0] lo_in; //LO Register in
  
   
  logic [31:0] alu_in1; //mux output of rs_data and rt_data
  logic [31:0] alu_in2; //mux output of rt_data and signed/unsigned imm to ALU

  logic IDen; //enable for decode stage
  logic [31:0] pc_ID;

  logic EXen; //enable for execute stage
  logic [31:0] pc_EX;
  logic [31:0] rs_data_EX;
  logic [31:0] rt_data_EX;
  logic [31:0] imm_EX;
  logic [4:0] wr_reg_EX;

  logic ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX;
  logic alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, load_stall_EX; 
  logic [1:0] memtoreg_EX, pcMuxSel_EX, store_sel_EX;
  logic [3:0] alu__sel_EX, mem_write_en_EX;
  logic [2:0] load_sel_EX, brcond_EX;

  logic MEMen; //enable for memory stage
  logic [31:0] pc_MEM;
  logic [31:0] alu__out_MEM;
  logic [31:0] rt_data_MEM;
  logic [31:0] imm_MEM;
  logic [4:0] wr_reg_MEM;

  logic ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM;
  logic alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, load_stall_MEM;
  logic [1:0] memtoreg_MEM, pcMuxSel_MEM, store_sel_MEM;
  logic [3:0] alu__sel_MEM, mem_write_en_MEM;
  logic [2:0] load_sel_MEM, brcond_MEM;

  logic WBen; //enable for WB stage
  logic [31:0] HIout_WB, LOout_WB, alu__out_WB;
  logic [31:0] rt_data_WB;
  logic [4:0] wr_reg_WB;
  logic ctrl_we_WB, ctrl_Sys_WB, ctrl_RI_WB, regdst_WB, jLink_en_WB;
  logic alusrc1_WB, alusrc2_WB, se_WB, hi_en_WB, lo_en_WB, load_stall_WB;
  logic [1:0] memtoreg_WB, pcMuxSel_WB, store_sel_WB;
  logic [3:0] alu__sel_WB, mem_write_en_WB;
  logic [2:0] load_sel_WB, brcond_WB;

  logic [31:0] rs_fwd; //forwarded value of rs
  logic [31:0] rt_fwd; //forwarded value of rt
  logic [2:0] fwd_rs_sel, fwd_rt_sel; //select bits for forwarding rs and rt
  logic [2:0] fwd_rs_sel_EX, fwd_rt_sel_EX;

}instruction;

////
//// The MIPS standalone processor module
////
////   clk          (input)  - The clock
////   inst_addr    (output) - Address of instruction to load
////   inst         (input)  - Instruction from memory
////   inst_excpt   (input)  - inst_addr not valid
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_data_out (input)  - Data from memory load
////   mem_write_en (output) - Memory write mask
////   mem_excpt    (input)  - mem_addr not valid
////   halted       (output) - Processor halted
////   reset        (input)  - Reset the processor
////   

module mips_core(/*AUTOARG*/
   // Outputs
   inst_addr, mem_addr, mem_data_in, mem_write_en, halted,
   // Inputs
   clk, inst_excpt, mem_excpt, inst, inst_1, inst_2,inst_3, mem_data_out, rst_b
   );
   
   parameter text_start  = 32'h00400000; /* Initial value of $pc */

   // Core Interface
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0] mem_addr;
   input  [31:0] inst, inst_1, inst_2, inst_3, mem_data_out;
   output [31:0] mem_data_in;
   output [3:0]  mem_write_en;
   output        halted;
   input         rst_b;


   // Internal signals
   //wire [31:0]   pc, nextpc, nextnextpc;

   wire [31:0]   r_v0;

   // Decode signals
   instruction instruc_1;
   instruction instruc_2;
   assign instruc_1.inst = inst;
   assign instruc_2.inst = inst_1;

   //wire [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
   //wire [5:0]    dcd_op, dcd_funct2;
   //wire [4:0]    dcd_rs, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
   //wire [15:0]   dcd_offset, dcd_imm;
   //wire [25:0]   dcd_target;
   //wire [19:0]   dcd_code;
   //wire          dcd_bczft;
  
   wire [31:0] load_data,load_data_WB; //loaded data
   wire [31:0] store_data; //data to store

   wire [31:0] newpc; //mux output for next state PC
   wire [31:0] stallpc;
   //wire [31:0] pcNextFinal; 
   //wire [31:0] inst_ID; //instruction progagated to decode stage

   // PC Management
   //register #(32, text_start) PCReg(pc, pcNextFinal, clk, ~internal_halt, rst_b);
   register #(32, text_start) PCReg(instruc_1.pc, stallpc, clk, ~internal_halt, rst_b);
   assign instruc_2.pc = instruc_1.pc + 4;
   assign inst_addr = instruc_1.pc[31:2];
   //register #(32, text_start+4) PCReg2(nextpc, newpc, clk,
   //                                    ~internal_halt, rst_b);
   //mux2to1 pickNextPC (pcNextFinal, nextpc, nextnextpc,(pcMuxSel[1]|pcMuxSel[0]));
   //add_const #(4) NextPCAdder(nextpc, pc);
   //add_const #(4) NextPCAdder(nextnextpc, nextpc, pcMuxSel);
   

   // Instruction decoding
   /*******Instruction 1*******/
   
   assign        instruc_1.dcd_op = instruc_1.inst_ID[31:26];    // Opcode
   assign        instruc_1.dcd_rs = instruc_1.inst_ID[25:21];    // rs field
   assign        instruc_1.dcd_rt = instruc_1.inst_ID[20:16];    // rt field
   assign        instruc_1.dcd_rd = instruc_1.inst_ID[15:11];    // rd field
   assign        instruc_1.dcd_shamt = instruc_1.inst_ID[10:6];  // Shift amount
   assign        instruc_1.dcd_bczft = instruc_1.inst_ID[16];    // bczt or bczf?
   assign        instruc_1.dcd_funct1 = instruc_1.inst_ID[4:0];  // Coprocessor 0 function field
   assign        instruc_1.dcd_funct2 = instruc_1.inst_ID[5:0];  // funct field; secondary opcode
   assign        instruc_1.dcd_offset = instruc_1.inst_ID[15:0]; // offset field
        // Sign-extended offset for branches
   assign        instruc_1.dcd_se_offset = { {14{instruc_1.dcd_offset[15]}}, instruc_1.dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        instruc_1.dcd_se_mem_offset = { {16{instruc_1.dcd_offset[15]}}, instruc_1.dcd_offset };
   assign        instruc_1.dcd_imm = instruc_1.inst_ID[15:0];        // immediate field
   assign        instruc_1.dcd_e_imm = { 16'h0, instruc_1.dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        instruc_1.dcd_se_imm = { {16{instruc_1.dcd_imm[15]}}, instruc_1.dcd_imm };
   assign        instruc_1.dcd_target = instruc_1.inst_ID[25:0];     // target field
   assign        instruc_1.dcd_code = instruc_1.inst_ID[25:6];       // Breakpoint code
   
   /*******Instruction 2*******/
   assign        instruc_2.dcd_op = instruc_2.inst_ID[31:26];    // Opcode
   assign        instruc_2.dcd_rs = instruc_2.inst_ID[25:21];    // rs field
   assign        instruc_2.dcd_rt = instruc_2.inst_ID[20:16];    // rt field
   assign        instruc_2.dcd_rd = instruc_2.inst_ID[15:11];    // rd field
   assign        instruc_2.dcd_shamt = instruc_2.inst_ID[10:6];  // Shift amount
   assign        instruc_2.dcd_bczft = instruc_2.inst_ID[16];    // bczt or bczf?
   assign        instruc_2.dcd_funct1 = instruc_2.inst_ID[4:0];  // Coprocessor 0 function field
   assign        instruc_2.dcd_funct2 = instruc_2.inst_ID[5:0];  // funct field; secondary opcode
   assign        instruc_2.dcd_offset = instruc_2.inst_ID[15:0]; // offset field
        // Sign-extended offset for branches
   assign        instruc_2.dcd_se_offset = { {14{instruc_2.dcd_offset[15]}}, instruc_2.dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        instruc_2.dcd_se_mem_offset = { {16{instruc_2.dcd_offset[15]}}, instruc_2.dcd_offset };
   assign        instruc_2.dcd_imm = instruc_2.inst_ID[15:0];        // immediate field
   assign        instruc_2.dcd_e_imm = { 16'h0, instruc_2.dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        instruc_2.dcd_se_imm = { {16{instruc_2.dcd_imm[15]}}, instruc_2.dcd_imm };
   assign        instruc_2.dcd_target = instruc_2.inst_ID[25:0];     // target field
   assign        instruc_2.dcd_code = instruc_2.inst_ID[25:6];       // Breakpoint code
   /****************************/

   // synthesis translate_off
   
   /*always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "=== Simulation Cycle %d ===", $time );
       //$display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
       //            pc, inst_ID, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted);
       // $display ("Store address: %d, %d, Store word: %d, ALUOUT: %d, en: %d", rt_data, mem_addr, mem_data_in, alu__out, mem_write_en);
       //$display ("HI: %x, LO: %x, hi_en_EX: %x, hi_en_WB:%x, lo_en_EX: %x, lo_en_WB: %x", hi_out, lo_out,hi_en_EX,hi_en_WB,lo_en_EX, lo_en_WB);
       //$display ("HIWB: %x, LOWB: %x", HIout_WB, LOout_WB);
       //$display ("D: wr_reg: %x, wr_data: %x, reg1: %x, reg2: %x, imm: %x, mem_en: %x", wr_reg, wr_data, dcd_rs, dcd_rt, imm, mem_en);
       //$display ("   fwd_rs_en: %x, fwd_rt_en: %x", fwd_rs_sel, fwd_rt_sel);
       //$display ("E: wr_reg_EX: %x, alu_in1: %x, alu_in2: %x, alu__out: %x ctrl_we_EX: %x, mem_EX: %x", wr_reg_EX, alu_in1, alu_in2, alu__out, ctrl_we_EX, mem_write_en_EX);
       //$display ("M: wr_reg_MEM: %x, alu__outMEM: %x, ctrl_we_MEM: %x, mem_MEM: %x", wr_reg_MEM, alu__out_MEM, ctrl_we_MEM, mem_write_en_MEM);
       //$display ("   mem_addr: %x, load_data: %x, load_sel: %x, mem_data_out: %x, store_data: %x", mem_addr, load_data, load_sel_EX, mem_data_out, store_data);
       //$display ("W: wr_reg_WB: %x, alu__out_wb: %x, ctrl_we_WB: %x, mem_WB: %x", wr_reg_WB, alu__out_WB, ctrl_we_WB, mem_write_en_WB);
       //$display ("sys: %x, rt_data: %x", ctrl_Sys, rt_data);
       //$display ("sys_WB: %x, rt_data: %x", ctrl_Sys_WB, rt_data_WB);
       //$display ("stall: %x, CDen: %x", stall, CDen);
       //$display ("Address: %h, Store: %h, Load:%h, en:%b", mem_addr, mem_data_in, mem_data_out, mem_write_en);
       //$display ("alu_in1: %d, alu_in2: %d, brcond: %b", alu_in1, alu_in2,brcond);
       //$display ("branchTrue: %b, pcMuxSel: %b, pcMuxSelFinal: %b", branchTrue, pcMuxSel, pcMuxSelFinal);
       //$display ("br_target: %x", br_target);
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   instruc_1.pc, instruc_1.inst_ID, instruc_1.dcd_op, instruc_1.dcd_rs, instruc_1.dcd_rt, instruc_1.dcd_rd, instruc_1.dcd_imm, instruc_1.dcd_funct2, ~rst_b, halted);
       $display ("D: wr_reg: %x, wr_data: %x, reg1: %x, reg2: %x, rs_data: %x, rt_data: %x, imm: %x, mem_en: %x, load_stall: %x, IDen: %b, IDclr: %b", instruc_1.wr_regNum, instruc_1.wr_data, instruc_1.dcd_rs, instruc_1.rt_regNum, instruc_1.rs_data, instruc_1.rt_data, instruc_1.imm, instruc_1.mem_en, instruc_1.load_stall, instruc_1.IDen, IDclr);
       $display ("   fwd_rs_en: %b, fwd_rt_en: %b", instruc_1.fwd_rs_sel, instruc_1.fwd_rt_sel);
       $display ("   rsfwd: %x, rtfwd: %x, fwd_rs_en_EX: %b, fwd_rt_en_EX: %b", instruc_1.rs_fwd, instruc_1.rt_fwd, instruc_1.fwd_rs_sel_EX, instruc_1.fwd_rt_sel_EX);
       $display ("E: wr_reg_EX: %x, alu_in1: %x, alu_in2: %x, alu__out: %x ctrl_we_EX: %x, mem_EX: %x", instruc_1.wr_reg_EX, instruc_1.alu_in1, instruc_1.alu_in2, instruc_1.alu__out, instruc_1.ctrl_we_EX, instruc_1.mem_write_en_EX);
       $display ("M: wr_reg_MEM: %x, alu__outMEM: %x, ctrl_we_MEM: %x, mem_MEM: %x", instruc_1.wr_reg_MEM, instruc_1.alu__out_MEM, instruc_1.ctrl_we_MEM, instruc_1.mem_write_en_MEM);
       $display ("   mem_addr: %x, load_data: %x, load_sel: %x, mem_data_out: %x, store_data: %x", {mem_addr, 2'b0}, load_data, instruc_1.load_sel_EX, mem_data_out, store_data);
       $display ("W: wr_reg_WB: %x, alu__out_wb: %x, ctrl_we_WB: %x, mem_WB: %x", instruc_1.wr_reg_WB, instruc_1.alu__out_WB, instruc_1.ctrl_we_WB, instruc_1.mem_write_en_WB);
       $display ("");
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   instruc_2.pc, instruc_2.inst_ID, instruc_2.dcd_op, instruc_2.dcd_rs, instruc_2.dcd_rt, instruc_2.dcd_rd, instruc_2.dcd_imm, instruc_2.dcd_funct2, ~rst_b, halted);
       $display ("D: wr_reg: %x, wr_data: %x, reg1: %x, reg2: %x, rs_data: %x, rt_data: %x, imm: %x, mem_en: %x, load_stall: %x", instruc_2.wr_regNum, instruc_2.wr_data, instruc_2.dcd_rs, instruc_2.rt_regNum, instruc_2.rs_data, instruc_2.rt_data, instruc_2.imm, instruc_2.mem_en, instruc_2.load_stall);
       $display ("   fwd_rs_en: %b, fwd_rt_en: %b", instruc_2.fwd_rs_sel, instruc_2.fwd_rt_sel);
       $display ("   rsfwd: %x, rtfwd: %x, fwd_rs_en_EX: %b, fwd_rt_en_EX: %b", instruc_2.rs_fwd, instruc_2.rt_fwd, instruc_2.fwd_rs_sel_EX, instruc_2.fwd_rt_sel_EX);
       $display ("E: wr_reg_EX: %x, alu_in1: %x, alu_in2: %x, alu__out: %x, ctrl_we_EX: %x, mem_EX: %x, EXen: %x", instruc_2.wr_reg_EX, instruc_2.alu_in1, instruc_2.alu_in2, instruc_2.alu__out, instruc_2.ctrl_we_EX, instruc_2.mem_write_en_EX, instruc_2.EXen);
       $display ("M: wr_reg_MEM: %x, alu__outMEM: %x, ctrl_we_MEM: %x, mem_MEM: %x", instruc_2.wr_reg_MEM, instruc_2.alu__out_MEM, instruc_2.ctrl_we_MEM, instruc_2.mem_write_en_MEM);
       $display ("   mem_addr: %x, load_data: %x, load_sel: %x, mem_data_out: %x, store_data: %x, rt_data_MEM: %x", {mem_addr, 2'b0}, load_data, instruc_2.load_sel_EX, mem_data_out, store_data, instruc_2.rt_data_MEM);
       $display ("W: wr_reg_WB: %x, alu__out_wb: %x, ctrl_we_WB: %x, mem_WB: %x", instruc_2.wr_reg_WB, instruc_2.alu__out_WB, instruc_2.ctrl_we_WB, instruc_2.mem_write_en_WB);
             $display ("halt: %x, ctrl_sys_WB: %x, rt_data_WB: %x", internal_halt, instruc_2.ctrl_Sys_WB, instruc_2.rt_data_WB);

       $display ("");
       $display ("");
     end
   end*/
   // synthesis translate_on

   // Let Verilog-Mode pipe wires through for us.  This is another example
   // of Verilog-Mode's power -- undeclared nets get AUTOWIREd up when we
   // run 'make auto'.
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   //wire [3:0]   alu__sel;   // From Decoder of mips_decode.v
   //wire     ctrl_RI;    // From Decoder of mips_decode.v
   //wire     ctrl_Sys;   // From Decoder of mips_decode.v
   //wire     ctrl_we;    // From Decoder of mips_decode.v
   // End of automatics

   //Added control signals
   //wire regdst;
   //wire jLink_en;
   //wire[2:0] brcond;
   //wire branchTrue;
   //wire [1:0] memtoreg;
   //wire alusrc1;
   //wire alusrc2;
   //wire se;
   //wire [3:0] mem_en; //memory write enable
   //wire [2:0] load_sel; //selects the type of load for the loader to perform
   //wire hi_en;
   //wire lo_en;
   //wire [1:0] store_sel; //selects the type of store for the storer to perform
   //wire load_stall; //check if instruction is a load or MF to signal a stall

   // Generate control signals
   /**********Instruction 1********/
   mips_decode Decoder_1(/*AUTOINST*/
          // Outputs
          .ctrl_we         (instruc_1.ctrl_we),
          .ctrl_Sys        (instruc_1.ctrl_Sys),
          .ctrl_RI         (instruc_1.ctrl_RI),
          .alu__sel        (instruc_1.alu__sel[3:0]),
          .regdst          (instruc_1.regdst),
          .pcMuxSel        (instruc_1.pcMuxSel),
          .jLink_en        (instruc_1.jLink_en),
          .memtoreg        (instruc_1.memtoreg),
          .alusrc1         (instruc_1.alusrc1),
          .alusrc2         (instruc_1.alusrc2),
          .se              (instruc_1.se),
          .mem_write_en    (instruc_1.mem_en),
          .hi_en           (instruc_1.hi_en),
          .lo_en           (instruc_1.lo_en),
          .load_sel        (instruc_1.load_sel),
          .brcond          (instruc_1.brcond),
          .store_sel       (instruc_1.store_sel),
          .load_stall      (instruc_1.load_stall),
          // Inputs
          .dcd_op          (instruc_1.dcd_op[5:0]),
          .dcd_funct2      (instruc_1.dcd_funct2[5:0]),
          .dcd_rt          (instruc_1.dcd_rt));
   
   /**********Instruction 2********/
   mips_decode Decoder_2(/*AUTOINST*/
          // Outputs
          .ctrl_we         (instruc_2.ctrl_we),
          .ctrl_Sys        (instruc_2.ctrl_Sys),
          .ctrl_RI         (instruc_2.ctrl_RI),
          .alu__sel        (instruc_2.alu__sel[3:0]),
          .regdst          (instruc_2.regdst),
          .pcMuxSel        (instruc_2.pcMuxSel),
          .jLink_en        (instruc_2.jLink_en),
          .memtoreg        (instruc_2.memtoreg),
          .alusrc1         (instruc_2.alusrc1),
          .alusrc2         (instruc_2.alusrc2),
          .se              (instruc_2.se),
          .mem_write_en    (instruc_2.mem_en),
          .hi_en           (instruc_2.hi_en),
          .lo_en           (instruc_2.lo_en),
          .load_sel        (instruc_2.load_sel),
          .brcond          (instruc_2.brcond),
          .store_sel       (instruc_2.store_sel),
          .load_stall      (instruc_2.load_stall),
          // Inputs
          .dcd_op          (instruc_2.dcd_op[5:0]),
          .dcd_funct2      (instruc_2.dcd_funct2[5:0]),
          .dcd_rt          (instruc_2.dcd_rt));
   /*******************************/
 
   // Register File
   // Instantiate the register file from regfile.v here.
   // Don't forget to hookup the "halted" signal to trigger the register dump 

/************Main Added Code**************************************************/

   //New wirings
   //wire [4:0] wr_reg; //input to write register
   //wire [31:0] imm; //signed or unsigned immediate
   //wire [31:0] wr_data; //data to write to register file
   //wire [31:0] wr_dataMem; //intermediate data to write to register file
   //wire [4:0] wr_regNum;//intermediate reg to write to 
   //wire [4:0] rt_regNum;//rt register to read from

   //wire [31:0] br_target; //branch target
   //wire [31:0] j_target; //unconditional jump target
   wire [31:0] hi_out; //HI Register out
   wire [31:0] lo_out; //LO Register out
   //wire [31:0] hi_in; //HI Register in
   //wire [31:0] lo_in; //LO Register in
   //wire [31:0] load_data; //loaded data
   //wire [31:0] store_data; //data to store
   
   //wire [31:0] alu_in1; //mux output of rs_data and rt_data
   //wire [31:0] alu_in2; //mux output of rt_data and signed/unsigned imm to ALU




   //Fetch (IF) stage for pc register
   wire IFen; //enable for IF stage
   //wire [31:0] stallpc; //either PC or PC+4 depending on stall conditions
   mux2to1 stallMux(stallpc, instruc_1.pc, instruc_1.pc+8, IFen);

   //Decode (ID) stage registers and wirings
   //wire IDen; //enable for decode stage
   //wire [31:0] pc_ID;
   wire IDclr;
   register pcID_1(instruc_1.pc_ID, instruc_1.pc, clk, instruc_1.IDen, rst_b);
   register pcID_2(instruc_2.pc_ID, instruc_2.pc, clk, instruc_2.IDen, rst_b);
   clearRegister irD_1(instruc_1.inst_ID, instruc_1.inst, clk, instruc_1.IDen, IDclr, rst_b);
   register irD_2(instruc_2.inst_ID, instruc_2.inst, clk, instruc_2.IDen, rst_b);

   //Execute (EX) stage registers
   //wire EXen; //enable for execute stage
   //wire [31:0] pc_EX;
   //wire [31:0] rs_data_EX;
   //wire [31:0] rt_data_EX;
   //wire [31:0] imm_EX;
   //wire [4:0] wr_reg_EX;
   
   /**********Instruction 1**********/
   register pcEX_1(instruc_1.pc_EX, instruc_1.pc_ID, clk, instruc_1.EXen, rst_b);
   register rsEX_1(instruc_1.rs_data_EX, instruc_1.rs_data, clk, instruc_1.EXen, rst_b);
   register rtEX_1(instruc_1.rt_data_EX, instruc_1.rt_data, clk, instruc_1.EXen, rst_b);
   register iEX_1(instruc_1.imm_EX, instruc_1.imm, clk, instruc_1.EXen, rst_b);
   register #(5) wrEX_1(instruc_1.wr_reg_EX, instruc_1.wr_regNum, clk, instruc_1.EXen, rst_b);
   register #(3) fwdrsEX_1(instruc_1.fwd_rs_sel_EX, instruc_1.fwd_rs_sel, clk, instruc_1.EXen, rst_b);
   register #(3) fwdrtEX_1(instruc_1.fwd_rt_sel_EX, instruc_1.fwd_rt_sel, clk, instruc_1.EXen, rst_b);

   //wire ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX;
   //wire alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, load_stall_EX; 
   //wire [1:0] memtoreg_EX, pcMuxSel_EX, store_sel_EX;
   //wire [3:0] alu__sel_EX, mem_write_en_EX;
   //wire [2:0] load_sel_EX, brcond_EX;
   cntlRegister cntlEX_1(instruc_1.ctrl_we_EX, instruc_1.ctrl_Sys_EX, instruc_1.ctrl_RI_EX,
                       instruc_1.regdst_EX, instruc_1.jLink_en_EX, instruc_1.alusrc1_EX,
                       instruc_1.alusrc2_EX, instruc_1.se_EX, instruc_1.hi_en_EX,
                       instruc_1.lo_en_EX, instruc_1.memtoreg_EX, instruc_1.pcMuxSel_EX,
                       instruc_1.alu__sel_EX, instruc_1.mem_write_en_EX, instruc_1.load_sel_EX,
                       instruc_1.brcond_EX, instruc_1.store_sel_EX, instruc_1.load_stall_EX,
                       //Inputs
                       instruc_1.ctrl_we, instruc_1.ctrl_Sys, instruc_1.ctrl_RI,
                       instruc_1.regdst, instruc_1.jLink_en, instruc_1.alusrc1,
                       instruc_1.alusrc2, instruc_1.se, instruc_1.hi_en, instruc_1.lo_en,
                       instruc_1.memtoreg, instruc_1.pcMuxSel, instruc_1.alu__sel,
                       instruc_1.mem_en, instruc_1.load_sel, instruc_1.brcond,
                       instruc_1.store_sel,instruc_1.load_stall, clk, instruc_1.EXen, rst_b);

   /**********Instruction 2**********/
   register pcEX_2(instruc_2.pc_EX, instruc_2.pc_ID, clk, instruc_2.EXen, rst_b);
   register rsEX_2(instruc_2.rs_data_EX, instruc_2.rs_data, clk, instruc_2.EXen, rst_b);
   register rtEX_2(instruc_2.rt_data_EX, instruc_2.rt_data, clk, instruc_2.EXen, rst_b);
   register iEX_2(instruc_2.imm_EX, instruc_2.imm, clk, instruc_2.EXen, rst_b);
   register #(5) wrEX_2(instruc_2.wr_reg_EX, instruc_2.wr_regNum, clk, instruc_2.EXen, rst_b);
   register #(3) fwdrsEX_2(instruc_2.fwd_rs_sel_EX, instruc_2.fwd_rs_sel, clk, instruc_2.EXen, rst_b);
   register #(3) fwdrtEX_2(instruc_2.fwd_rt_sel_EX, instruc_2.fwd_rt_sel, clk, instruc_2.EXen, rst_b);

   //wire ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX;
   //wire alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, load_stall_EX; 
   //wire [1:0] memtoreg_EX, pcMuxSel_EX, store_sel_EX;
   //wire [3:0] alu__sel_EX, mem_write_en_EX;
   //wire [2:0] load_sel_EX, brcond_EX;
   cntlRegister cntlEX_2(instruc_2.ctrl_we_EX, instruc_2.ctrl_Sys_EX, instruc_2.ctrl_RI_EX,
                       instruc_2.regdst_EX, instruc_2.jLink_en_EX, instruc_2.alusrc1_EX,
                       instruc_2.alusrc2_EX, instruc_2.se_EX, instruc_2.hi_en_EX,
                       instruc_2.lo_en_EX, instruc_2.memtoreg_EX, instruc_2.pcMuxSel_EX,
                       instruc_2.alu__sel_EX, instruc_2.mem_write_en_EX, instruc_2.load_sel_EX,
                       instruc_2.brcond_EX, instruc_2.store_sel_EX, instruc_2.load_stall_EX,
                       //Inputs
                       instruc_2.ctrl_we, instruc_2.ctrl_Sys, instruc_2.ctrl_RI,
                       instruc_2.regdst, instruc_2.jLink_en, instruc_2.alusrc1,
                       instruc_2.alusrc2, instruc_2.se, instruc_2.hi_en, instruc_2.lo_en,
                       instruc_2.memtoreg, instruc_2.pcMuxSel, instruc_2.alu__sel,
                       instruc_2.mem_en, instruc_2.load_sel, instruc_2.brcond,
                       instruc_2.store_sel,instruc_2.load_stall, clk, instruc_2.EXen, rst_b);

   //Memory (MEM) stage registers
   //wire MEMen; //enable for memory stage
   //wire [31:0] pc_MEM;
   //wire [31:0] alu__out_MEM;
   //wire [31:0] rt_data_MEM;
   //wire [31:0] imm_MEM;
   //wire [4:0] wr_reg_MEM;
   /************Instruction 1**********/
   assign instruc_1.MEMen = 1; //MEM stage never disabled
   register pcMEM_1(instruc_1.pc_MEM, instruc_1.pc_EX, clk, instruc_1.MEMen, rst_b);
   register aluMEM_1(instruc_1.alu__out_MEM, instruc_1.alu__out, clk, instruc_1.MEMen, rst_b);
   register rtMEM_1(instruc_1.rt_data_MEM, instruc_1.rt_fwd, clk, instruc_1.MEMen, rst_b);
   register iMEM_1(instruc_1.imm_MEM, instruc_1.imm_EX, clk, instruc_1.MEMen, rst_b);
   register #(5) wrMEM_1(instruc_1.wr_reg_MEM, instruc_1.wr_reg_EX, clk, instruc_1.MEMen, rst_b);

   //wire ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM;
   //wire alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, load_stall_MEM;
   //wire [1:0] memtoreg_MEM, pcMuxSel_MEM, store_sel_MEM;
   //wire [3:0] alu__sel_MEM, mem_write_en_MEM;
   //wire [2:0] load_sel_MEM, brcond_MEM;
   cntlRegister cntlMEM_1(instruc_1.ctrl_we_MEM, instruc_1.ctrl_Sys_MEM, instruc_1.ctrl_RI_MEM,
                        instruc_1.regdst_MEM, instruc_1.jLink_en_MEM, instruc_1.alusrc1_MEM,
                        instruc_1.alusrc2_MEM, instruc_1.se_MEM, instruc_1.hi_en_MEM,
                        instruc_1.lo_en_MEM, instruc_1.memtoreg_MEM, instruc_1.pcMuxSel_MEM,
                        instruc_1.alu__sel_MEM, instruc_1.mem_write_en_MEM, instruc_1.load_sel_MEM,
                        instruc_1.brcond_MEM, instruc_1.store_sel_MEM,instruc_1.load_stall_MEM,
                        //Inputs
                        instruc_1.ctrl_we_EX, instruc_1.ctrl_Sys_EX, instruc_1.ctrl_RI_EX,
                        instruc_1.regdst_EX, instruc_1.jLink_en_EX, instruc_1.alusrc1_EX,
                        instruc_1.alusrc2_EX, instruc_1.se_EX, instruc_1.hi_en_EX,
                        instruc_1.lo_en_EX, instruc_1.memtoreg_EX, instruc_1.pcMuxSel_EX,
                        instruc_1.alu__sel_EX, instruc_1.mem_write_en_EX, instruc_1.load_sel_EX,
                        instruc_1.brcond_EX, instruc_1.store_sel_EX, instruc_1.load_stall_EX,
                        clk, instruc_1.MEMen, rst_b);
   
   /************Instruction 2**********/
   assign instruc_2.MEMen = 1; //MEM stage never disabled
   register pcMEM_2(instruc_2.pc_MEM, instruc_2.pc_EX, clk, instruc_2.MEMen, rst_b);
   register aluMEM_2(instruc_2.alu__out_MEM, instruc_2.alu__out, clk, instruc_2.MEMen, rst_b);
   register rtMEM_2(instruc_2.rt_data_MEM, instruc_2.rt_fwd, clk, instruc_2.MEMen, rst_b);
   register iMEM_2(instruc_2.imm_MEM, instruc_2.imm_EX, clk, instruc_2.MEMen, rst_b);
   register #(5) wrMEM_2(instruc_2.wr_reg_MEM, instruc_2.wr_reg_EX, clk, instruc_2.MEMen, rst_b);

   //wire ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM;
   //wire alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, load_stall_MEM;
   //wire [1:0] memtoreg_MEM, pcMuxSel_MEM, store_sel_MEM;
   //wire [3:0] alu__sel_MEM, mem_write_en_MEM;
   //wire [2:0] load_sel_MEM, brcond_MEM;
   cntlRegister cntlMEM_2(instruc_2.ctrl_we_MEM, instruc_2.ctrl_Sys_MEM, instruc_2.ctrl_RI_MEM,
                        instruc_2.regdst_MEM, instruc_2.jLink_en_MEM, instruc_2.alusrc1_MEM,
                        instruc_2.alusrc2_MEM, instruc_2.se_MEM, instruc_2.hi_en_MEM,
                        instruc_2.lo_en_MEM, instruc_2.memtoreg_MEM, instruc_2.pcMuxSel_MEM,
                        instruc_2.alu__sel_MEM, instruc_2.mem_write_en_MEM, instruc_2.load_sel_MEM,
                        instruc_2.brcond_MEM, instruc_2.store_sel_MEM,instruc_2.load_stall_MEM,
                       //Inputs
                        instruc_2.ctrl_we_EX, instruc_2.ctrl_Sys_EX, instruc_2.ctrl_RI_EX,
                        instruc_2.regdst_EX, instruc_2.jLink_en_EX, instruc_2.alusrc1_EX,
                        instruc_2.alusrc2_EX, instruc_2.se_EX, instruc_2.hi_en_EX,
                        instruc_2.lo_en_EX, instruc_2.memtoreg_EX, instruc_2.pcMuxSel_EX,
                        instruc_2.alu__sel_EX, instruc_2.mem_write_en_EX, instruc_2.load_sel_EX,
                        instruc_2.brcond_EX, instruc_2.store_sel_EX, instruc_2.load_stall_EX,
                        clk, instruc_2.MEMen, rst_b);
   /************************************/

   //Writeback stage registers
   //wire WBen; //enable for WB stage
   //wire [31:0] HIout_WB, LOout_WB, load_data_WB, alu__out_WB;
   //wire [31:0] rt_data_WB;
   //wire [4:0] wr_reg_WB;
   /**********Instruction 1********/
   assign instruc_1.WBen = 1; //WB stage never disabled
   //register MDRw_1(instruc_1.load_data_WB, instruc_1.load_data, clk, instruc_1.WBen, rst_b);
   register Aoutw_1(instruc_1.alu__out_WB, instruc_1.alu__out_MEM, clk, instruc_1.WBen, rst_b);
   register HIwb_1(instruc_1.HIout_WB, instruc_1.hi_out, clk, instruc_1.WBen, rst_b); //holds HI val in WB register (may need to have its own en)
   register LOwb_1(instruc_1.LOout_WB, instruc_1.lo_out, clk, instruc_1.WBen, rst_b); //holds LO val in WB register (may need to have its own en)
   register rtWB_1(instruc_1.rt_data_WB, instruc_1.rt_data_MEM, clk, instruc_1.WBen, rst_b);
   register #(5) wrWB_1(instruc_1.wr_reg_WB, instruc_1.wr_reg_MEM, clk, instruc_1.WBen, rst_b);

   //wire ctrl_we_WB, ctrl_Sys_WB, ctrl_RI_WB, regdst_WB, jLink_en_WB;
   //wire alusrc1_WB, alusrc2_WB, se_WB, hi_en_WB, lo_en_WB, load_stall_WB;
   //wire [1:0] memtoreg_WB, pcMuxSel_WB, store_sel_WB;
   //wire [3:0] alu__sel_WB, mem_write_en_WB;
   //wire [2:0] load_sel_WB, brcond_WB;
   cntlRegister cntlWB_1(instruc_1.ctrl_we_WB, instruc_1.ctrl_Sys_WB, instruc_1.ctrl_RI_WB,
                        instruc_1.regdst_WB, instruc_1.jLink_en_WB, instruc_1.alusrc1_WB,
                        instruc_1.alusrc2_WB, instruc_1.se_WB, instruc_1.hi_en_WB,
                        instruc_1.lo_en_WB, instruc_1.memtoreg_WB, instruc_1.pcMuxSel_WB,
                        instruc_1.alu__sel_WB, instruc_1.mem_write_en_WB, instruc_1.load_sel_WB,
                        instruc_1.brcond_WB, instruc_1.store_sel_WB,instruc_1.load_stall_WB,
                       //Inputs
                        instruc_1.ctrl_we_MEM, instruc_1.ctrl_Sys_MEM, instruc_1.ctrl_RI_MEM,
                        instruc_1.regdst_MEM, instruc_1.jLink_en_MEM, instruc_1.alusrc1_MEM,
                        instruc_1.alusrc2_MEM, instruc_1.se_MEM, instruc_1.hi_en_MEM,
                        instruc_1.lo_en_MEM, instruc_1.memtoreg_MEM, instruc_1.pcMuxSel_MEM,
                        instruc_1.alu__sel_MEM, instruc_1.mem_write_en_MEM, instruc_1.load_sel_MEM,
                        instruc_1.brcond_MEM, instruc_1.store_sel_MEM, instruc_1.load_stall_MEM,
                        clk, instruc_1.WBen, rst_b);

   /**********Instruction 2**********/
   assign instruc_2.WBen = 1; //WB stage never disabled
   //register MDRw_2(instruc_2.load_data_WB, instruc_2.load_data, clk, instruc_2.WBen, rst_b);
   register Aoutw_2(instruc_2.alu__out_WB, instruc_2.alu__out_MEM, clk, instruc_2.WBen, rst_b);
   register HIwb_2(instruc_2.HIout_WB, instruc_2.hi_out, clk, instruc_2.WBen, rst_b); //holds HI val in WB register (may need to have its own en)
   register LOwb_2(instruc_2.LOout_WB, instruc_2.lo_out, clk, instruc_2.WBen, rst_b); //holds LO val in WB register (may need to have its own en)
   register rtWB_2(instruc_2.rt_data_WB, instruc_2.rt_data_MEM, clk, instruc_2.WBen, rst_b);
   register #(5) wrWB_2(instruc_2.wr_reg_WB, instruc_2.wr_reg_MEM, clk, instruc_2.WBen, rst_b);

   //wire ctrl_we_WB, ctrl_Sys_WB, ctrl_RI_WB, regdst_WB, jLink_en_WB;
   //wire alusrc1_WB, alusrc2_WB, se_WB, hi_en_WB, lo_en_WB, load_stall_WB;
   //wire [1:0] memtoreg_WB, pcMuxSel_WB, store_sel_WB;
   //wire [3:0] alu__sel_WB, mem_write_en_WB;
   //wire [2:0] load_sel_WB, brcond_WB;
   cntlRegister cntlWB_2(instruc_2.ctrl_we_WB, instruc_2.ctrl_Sys_WB, instruc_2.ctrl_RI_WB,
                        instruc_2.regdst_WB, instruc_2.jLink_en_WB, instruc_2.alusrc1_WB,
                        instruc_2.alusrc2_WB, instruc_2.se_WB, instruc_2.hi_en_WB,
                        instruc_2.lo_en_WB, instruc_2.memtoreg_WB, instruc_2.pcMuxSel_WB,
                        instruc_2.alu__sel_WB, instruc_2.mem_write_en_WB, instruc_2.load_sel_WB,
                        instruc_2.brcond_WB, instruc_2.store_sel_WB,instruc_2.load_stall_WB,
                       //Inputs
                        instruc_2.ctrl_we_MEM, instruc_2.ctrl_Sys_MEM, instruc_2.ctrl_RI_MEM,
                        instruc_2.regdst_MEM, instruc_2.jLink_en_MEM, instruc_2.alusrc1_MEM,
                        instruc_2.alusrc2_MEM, instruc_2.se_MEM, instruc_2.hi_en_MEM,
                        instruc_2.lo_en_MEM, instruc_2.memtoreg_MEM, instruc_2.pcMuxSel_MEM,
                        instruc_2.alu__sel_MEM, instruc_2.mem_write_en_MEM, instruc_2.load_sel_MEM,
                        instruc_2.brcond_MEM, instruc_2.store_sel_MEM, instruc_2.load_stall_MEM,
                        clk, instruc_2.WBen, rst_b);
    /*******************************/

   //check for RAW hazard and Stall
   wire CDen;
   wire [2:0] CDAmt;
   stallDetector sD(instruc_1.pc_ID, instruc_1.pc_EX,
                    instruc_1.wr_regNum, instruc_1.wr_reg_EX, instruc_1.wr_reg_MEM, instruc_1.rt_regNum, instruc_1.dcd_rs,
                    instruc_1.mem_en,
                    instruc_1.ctrl_we, instruc_1.ctrl_we_EX, instruc_1.regdst,
                    instruc_2.pc_ID, instruc_2.pc_EX,
                    instruc_2.wr_regNum, instruc_2.wr_reg_EX, instruc_2.wr_reg_MEM, instruc_2.rt_regNum, instruc_2.dcd_rs,
                    instruc_2.mem_en,
                    instruc_2.ctrl_we, instruc_2.ctrl_we_EX, instruc_2.regdst,
                    1'b0, instruc_2.stall, instruc_1.load_stall, instruc_1.load_stall_EX, instruc_2.load_stall, instruc_2.load_stall_EX,
                    instruc_1.store_sel,instruc_2.store_sel,
                    IFen, instruc_1.IDen, instruc_1.EXen, , instruc_2.IDen, instruc_2.EXen,
                    IDclr, CDen, CDAmt);
   countdownReg cdReg(CDen, clk, rst_b,
                      CDAmt,
                      instruc_2.stall);

   //Register file
   regfile2_forward RegFile(instruc_1.rs_data, instruc_1.rt_data,
                            instruc_2.rs_data, instruc_2.rt_data,

                            instruc_1.dcd_rs, instruc_1.rt_regNum,
                            instruc_1.wr_reg, instruc_1.wr_data, instruc_1.ctrl_we_WB,
                            instruc_2.dcd_rs, instruc_2.rt_regNum,
                            instruc_2.wr_reg, instruc_2.wr_data, instruc_2.ctrl_we_WB,
                            clk, rst_b, halted);

   //HI, LO registers
   register2Input #(32,0) hiReg(hi_out, instruc_1.rs_fwd, instruc_2.rs_fwd, clk, {instruc_1.hi_en_EX, instruc_2.hi_en_EX}, rst_b);
   register2Input #(32,0) loReg(lo_out, instruc_1.rs_fwd, instruc_2.rs_fwd, clk, {instruc_1.lo_en_EX, instruc_2.lo_en_EX}, rst_b);

   forwardData fwd(instruc_1.wr_reg_EX, instruc_1.wr_reg_MEM, instruc_1.rt_regNum, instruc_1.dcd_rs,
                   instruc_2.wr_reg_EX, instruc_2.wr_reg_MEM, instruc_2.rt_regNum, instruc_2.dcd_rs,
                   instruc_1.ctrl_we_EX, instruc_1.ctrl_we_MEM, instruc_2.ctrl_we_EX, instruc_2.ctrl_we_MEM,
                   instruc_1.fwd_rs_sel, instruc_1.fwd_rt_sel, instruc_2.fwd_rs_sel, instruc_2.fwd_rt_sel);

   //To read from / write to memory
   loader loader(load_data, instruc_1.imm_MEM[15:0], instruc_2.imm_MEM[15:0], mem_data_out,
                instruc_1.load_sel_MEM, instruc_2.load_sel_MEM,
                instruc_1.alu__out_MEM, instruc_2.alu__out_MEM); //operates on data loaded from memory

   storer storer(store_data, mem_write_en,
                instruc_1.rt_data_MEM, instruc_2.rt_data_MEM,
                instruc_1.store_sel_MEM, instruc_2.store_sel_MEM,
                instruc_1.alu__out_MEM, instruc_2.alu__out_MEM,
                instruc_1.mem_write_en_MEM, instruc_2.mem_write_en_MEM); //operates on data to write to memory

   /********PIPELINE1 MUXES**********/
   mux2to1 #(5) regDest(instruc_1.wr_regNum, instruc_1.dcd_rt, instruc_1.dcd_rd, instruc_1.regdst); //register to write to
   mux2to1 #(5) regRt(instruc_1.rt_regNum, instruc_1.dcd_rt, 5'd2, instruc_1.ctrl_Sys); //rt reg to read from (for syscalls)   

   //Determines inputs to ALU
   mux2to1 aluSrc1(instruc_1.alu_in1, instruc_1.rs_fwd, instruc_1.rt_fwd, instruc_1.alusrc1_EX); //ALUSrc1
   mux2to1 aluSrc2(instruc_1.alu_in2, instruc_1.rt_fwd, instruc_1.imm_EX, instruc_1.alusrc2_EX); //ALUSrc2
   mux2to1 signext_1(instruc_1.imm, instruc_1.dcd_e_imm, instruc_1.dcd_se_imm,
            instruc_1.se); //Zero extend or sign extend immediate

   //rs and rt forwarding
   mux5to1 fwdrs_1(instruc_1.rs_fwd, instruc_1.rs_data_EX, instruc_2.alu__out_MEM, instruc_1.alu__out_MEM, instruc_2.wr_dataMem, instruc_1.wr_dataMem, instruc_1.fwd_rs_sel_EX);
   mux5to1 fwdrt_1(instruc_1.rt_fwd, instruc_1.rt_data_EX, instruc_2.alu__out_MEM, instruc_1.alu__out_MEM, instruc_2.wr_dataMem, instruc_1.wr_dataMem, instruc_1.fwd_rt_sel_EX);
   
   //Wirings to memory module
   mux4to1 memToReg_1(instruc_1.wr_dataMem, instruc_1.alu__out_WB, load_data_WB,
                      instruc_1.HIout_WB, instruc_1.LOout_WB, instruc_1.memtoreg_WB);



   //Set wr_data and wr_reg when there is a jump/branch with link
   mux2to1 dataToReg_1(instruc_1.wr_data, instruc_1.wr_dataMem, instruc_1.pc+4, instruc_1.jLink_en_WB); 
   mux2to1 #(5)regNumber_1(instruc_1.wr_reg, instruc_1.wr_reg_WB, 5'd31, instruc_1.jLink_en_WB);


   /********PIPELINE2 MUXES**********/
   mux2to1 #(5) regDest_2(instruc_2.wr_regNum, instruc_2.dcd_rt, instruc_2.dcd_rd, instruc_2.regdst); //register to write to
   mux2to1 #(5) regRt_2(instruc_2.rt_regNum, instruc_2.dcd_rt, 5'd2, instruc_2.ctrl_Sys); //rt reg to read from (for syscalls)   

   //Determines inputs to ALU
   mux2to1 aluSrc1_2(instruc_2.alu_in1, instruc_2.rs_fwd, instruc_2.rt_fwd, instruc_2.alusrc1_EX); //ALUSrc1
   mux2to1 aluSrc2_2(instruc_2.alu_in2, instruc_2.rt_fwd, instruc_2.imm_EX, instruc_2.alusrc2_EX); //ALUSrc2
   mux2to1 signext_2(instruc_2.imm, instruc_2.dcd_e_imm, instruc_2.dcd_se_imm,
                     instruc_2.se); //Zero extend or sign extend immediate

   //rs and rt forwarding
   mux5to1 fwdrs_2(instruc_2.rs_fwd, instruc_2.rs_data_EX, instruc_2.alu__out_MEM, instruc_1.alu__out_MEM, instruc_2.wr_dataMem, instruc_1.wr_dataMem, instruc_2.fwd_rs_sel_EX);
   mux5to1 fwdrt_2(instruc_2.rt_fwd, instruc_2.rt_data_EX, instruc_2.alu__out_MEM, instruc_1.alu__out_MEM, instruc_2.wr_dataMem, instruc_1.wr_dataMem, instruc_2.fwd_rt_sel_EX);

   mux4to1 memToReg_2(instruc_2.wr_dataMem, instruc_2.alu__out_WB, load_data_WB,
                      instruc_2.HIout_WB, instruc_2.LOout_WB, instruc_2.memtoreg_WB);

   mux2to1 dataToReg_2(instruc_2.wr_data, instruc_2.wr_dataMem, instruc_2.pc+4, instruc_2.jLink_en_WB); 
   mux2to1 #(5)regNumber_2(instruc_2.wr_reg, instruc_2.wr_reg_WB, 5'd31, instruc_2.jLink_en_WB);



/*****************************************************************************/

   //Mux for next state PC
   //mux4to1 pcMux(newpc, stallpc, br_target, rs_data, j_target, pcMuxSelFinal); //chooses next PC depending on jump or branch
   //adder brtarget(br_target, pc + 4, (imm << 2), 1'b0); //get branch target
   //concat conc(j_target, pc, dcd_target); //get jump target
   //pcSelector choosePcMuxSel(pcMuxSelFinal,pcMuxSel,branchTrue); //chooses PC on whether branch condition is met

   //propogate load_data to WB stage
   register MDRw(load_data_WB, load_data, clk, instruc_2.WBen | instruc_1.WBen, rst_b);

   //sets values for mem_addr and mem_data_in if there is a store
   setMemValues setMemForStore(mem_addr,mem_data_in,
                    instruc_1.alu__out_MEM, instruc_2.alu__out_MEM, store_data,
                    instruc_1.mem_write_en_MEM, instruc_2.mem_write_en_MEM,
                    instruc_1.load_sel_MEM, instruc_2.load_sel_MEM);

   // Execute
   mips_ALU ALU1(.alu__out(instruc_1.alu__out),
                .branchTrue(instruc_1.branchTrue), 
                .alu__op1(instruc_1.alu_in1),
                .alu__op2(instruc_1.alu_in2),
                .alu__sel(instruc_1.alu__sel_EX),
                .brcond(instruc_1.brcond_EX));

   mips_ALU ALU2(.alu__out(instruc_2.alu__out),
                .branchTrue(instruc_2.branchTrue), 
                .alu__op1(instruc_2.alu_in1),
                .alu__op2(instruc_2.alu_in2),
                .alu__sel(instruc_2.alu__sel_EX),
                .brcond(instruc_2.brcond_EX));

 
   // Miscellaneous stuff (Exceptions, syscalls, and halt)
   exception_unit EU_1(.exception_halt(instruc_1.exception_halt), .pc(instruc_1.pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(instruc_1.load_ex_regs),
                     .load_bva(instruc_1.load_bva), .load_bva_sel(instruc_1.load_bva_sel),
                     .cause(instruc_1.cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(instruc_1.pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   exception_unit EU_2(.exception_halt(instruc_2.exception_halt), .pc(instruc_2.pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(instruc_2.load_ex_regs),
                     .load_bva(instruc_2.load_bva), .load_bva_sel(instruc_2.load_bva_sel),
                     .cause(instruc_2.cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(instruc_2.pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   assign r_v0 = (instruc_2.ctrl_Sys_WB) ? instruc_2.rt_data_WB : instruc_1.rt_data_WB; // rt_data for syscall is data from $v0

   syscall_unit SU_1(.syscall_halt(instruc_1.syscall_halt), .pc(instruc_1.pc), .clk(clk), .Sys(instruc_1.ctrl_Sys_WB),
                   .r_v0(r_v0), .rst_b(rst_b));
   syscall_unit SU_2(.syscall_halt(instruc_2.syscall_halt), .pc(instruc_2.pc), .clk(clk), .Sys(instruc_2.ctrl_Sys_WB),
                   .r_v0(r_v0), .rst_b(rst_b));

   assign        internal_halt = instruc_1.exception_halt | instruc_2.exception_halt | instruc_1.syscall_halt |instruc_2.syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   
   register #(32, 0) EPCReg_1(instruc_1.epc, instruc_1.pc, clk, instruc_1.load_ex_regs, rst_b);
   register #(32, 0) EPCReg_2(instruc_2.epc, instruc_2.pc, clk, instruc_2.load_ex_regs, rst_b);

   register #(32, 0) CauseReg_1(instruc_1.cause,
                              {25'b0, instruc_1.cause_code, 2'b0}, 
                              clk, instruc_1.load_ex_regs, rst_b);
   register #(32, 0) CauseReg_2(instruc_2.cause,
                              {25'b0, instruc_2.cause_code, 2'b0}, 
                              clk, instruc_2.load_ex_regs, rst_b);

   register #(32, 0) BadVAddrReg_1(instruc_1.bad_v_addr, instruc_1.pc, clk, instruc_1.load_bva, rst_b);
   register #(32, 0) BadVAddrReg_2(instruc_2.bad_v_addr, instruc_2.pc, clk, instruc_2.load_bva, rst_b);

endmodule // mips_core


////
//// adder
////
//// out (output) - adder result
//// in1 (input)  - Operand1
//// in2 (input)  - Operand2
//// sub (input)  - Subtract?
////
module adder(out, in1, in2, sub);
   output [31:0] out;
   input [31:0]  in1, in2;
   input         sub;

   assign        out = sub?(in1 - in2):(in1 + in2);

endmodule // adder


////
//// add_const: An adder that adds a fixed constant value
////
//// out (output) - adder result
//// in  (input)  - Operand
////
module add_const(out, in);//, sel);

   parameter add_value = 1;

   output logic [31:0] out;
   input logic  [31:0] in;
   //input logic [1:0] sel;
   
   assign out = in+add_value;
   /*always_comb begin
      if (sel == 2'b00)
        out = in + add_value;
      else
        out = in - add_value;
    end*/

endmodule // adder



////
//// pcSelector: sets the select bits to choose the next PC
////
//// pcMuxSelFinal (output) - select bits
//// pcMuxSel      (input)  - select bits without considering branch condition
//// branchTrue    (input)  - whether the branch condition was met
////
module pcSelector #(parameter width = 2) (
      output logic [width - 1:0] pcMuxSelFinal,
      input logic [width - 1:0] pcMuxSel,
      input logic branchTrue);

    always_comb
      begin
        pcMuxSelFinal=2'b00;
        if (branchTrue == 1'b0 && pcMuxSel == 2'b01)
          pcMuxSelFinal = 2'b00;
        else
          pcMuxSelFinal = pcMuxSel;
      end

endmodule

////
//// concat: concatenates the top 4 bits of PC and the bottom 26 bits 
////         of the current instruction for unconditional jumps shifted
///          left by 2
////
//// j_target (output)  - data chosen to be outputted
//// cur_pc (input)     - current PC
//// dcd_target (input) - bottom 26 bits of instruction
////
module concat (
      output logic [31:0] j_target,
      input logic [31:0] cur_pc,
      input logic [25:0] dcd_target);

    assign j_target = {cur_pc[31:28], dcd_target[25:0], 2'b00};

endmodule


// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
