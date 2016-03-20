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
   clk, inst_excpt, mem_excpt, inst, mem_data_out, rst_b
   );
   
   parameter text_start  = 32'h00400000; /* Initial value of $pc */

   // Core Interfae
   input         clk, inst_excpt, mem_excpt;
   output [29:0] inst_addr;
   output [29:0] mem_addr;
   input  [31:0] inst, mem_data_out;
   output [31:0] mem_data_in;
   output [3:0]  mem_write_en;
   output        halted;
   input         rst_b;


   // Internal signals
   wire [31:0]   pc, nextpc, nextnextpc;
   wire          exception_halt, syscall_halt, internal_halt;
   wire          load_epc, load_bva, load_bva_sel;
   wire [31:0]   rt_data, rs_data, rd_data, alu__out, r_v0;
   wire [31:0]   epc, cause, bad_v_addr;
   wire [4:0]    cause_code;

   // Decode signals
   wire [31:0]   dcd_se_imm, dcd_se_offset, dcd_e_imm, dcd_se_mem_offset;
   wire [5:0]    dcd_op, dcd_funct2;
   wire [4:0]    dcd_rs, dcd_funct1, dcd_rt, dcd_rd, dcd_shamt;
   wire [15:0]   dcd_offset, dcd_imm;
   wire [25:0]   dcd_target;
   wire [19:0]   dcd_code;
   wire          dcd_bczft;
   
   wire[1:0] pcMuxSel;
   wire[1:0] pcMuxSelFinal;

   wire [31:0] newpc; //mux output for next state PC
   //wire [31:0] pcNextFinal;
   wire [31:0] inst_ID; //instruction progagated to decode stage

   // PC Management
   //register #(32, text_start) PCReg(pc, pcNextFinal, clk, ~internal_halt, rst_b);
   register #(32, text_start) PCReg(pc, newpc, clk, ~internal_halt, rst_b);
   register #(32, text_start+4) PCReg2(nextpc, newpc, clk,
                                       ~internal_halt, rst_b);
   //mux2to1 pickNextPC (pcNextFinal, nextpc, nextnextpc,(pcMuxSel[1]|pcMuxSel[0]));
   add_const #(4) NextPCAdder(nextpc, pc);
   //add_const #(4) NextPCAdder(nextnextpc, nextpc, pcMuxSel);
   assign        inst_addr = pc[31:2];

   // Instruction decoding
   assign        dcd_op = inst_ID[31:26];    // Opcode
   assign        dcd_rs = inst_ID[25:21];    // rs field
   assign        dcd_rt = inst_ID[20:16];    // rt field
   assign        dcd_rd = inst_ID[15:11];    // rd field
   assign        dcd_shamt = inst_ID[10:6];  // Shift amount
   assign        dcd_bczft = inst_ID[16];    // bczt or bczf?
   assign        dcd_funct1 = inst_ID[4:0];  // Coprocessor 0 function field
   assign        dcd_funct2 = inst_ID[5:0];  // funct field; secondary opcode
   assign        dcd_offset = inst_ID[15:0]; // offset field
        // Sign-extended offset for branches
   assign        dcd_se_offset = { {14{dcd_offset[15]}}, dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        dcd_se_mem_offset = { {16{dcd_offset[15]}}, dcd_offset };
   assign        dcd_imm = inst_ID[15:0];        // immediate field
   assign        dcd_e_imm = { 16'h0, dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
   assign        dcd_target = inst_ID[25:0];     // target field
   assign        dcd_code = inst_ID[25:6];       // Breakpoint code

   // synthesis translate_off
   
   always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
       $display ( "=== Simulation Cycle %d ===", $time );
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   pc_ID, inst_ID, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted);
      // $display ("Store address: %d, %d, Store word: %d, ALUOUT: %d, en: %d", rt_data, mem_addr, mem_data_in, alu__out, mem_write_en);
       //$display ("HI: %x, LO: %x, hi_en_EX: %x, hi_en_WB:%x, lo_en_EX: %x, lo_en_WB: %x", hi_out, lo_out,hi_en_EX,hi_en_WB,lo_en_EX, lo_en_WB);
       //$display ("HIWB: %x, LOWB: %x", HIout_WB, LOout_WB);
       $display ("F: pc: %x, inst_addr: %x", pc, inst_addr);
       $display ("D: IDen: %x, wr_reg: %x, wr_data: %x, reg1: %x, reg2: %x, imm: %x, mem_en: %x, rs_data: %x", IDen, wr_regNum, wr_data, dcd_rs, dcd_rt, imm, mem_en, rs_data);
       //$display ("   fwd_rs_en: %x, fwd_rt_en: %x", fwd_rs_sel, fwd_rt_sel);
       $display ("E: EXen, %x, wr_reg_EX: %x, alu_in1: %x, alu_in2: %x, alu__out: %x, rs_data_EX: %x, ctrl_we_EX: %x, mem_EX: %x", EXen, wr_reg_EX, alu_in1, alu_in2, alu__out, rs_data_EX, ctrl_we_EX, mem_write_en_EX);
       $display ("M: wr_reg_MEM: %x, alu__outMEM: %x, mem_addr: %x, ctrl_we_MEM: %x, mem_MEM: %x", wr_reg_MEM, alu__out_MEM, mem_addr, ctrl_we_MEM, mem_write_en_MEM);
       //$display ("   mem_addr: %x, load_data: %x, load_sel: %x, mem_data_out: %x, store_data: %x", mem_addr, load_data, load_sel_EX, mem_data_out, store_data);
       $display ("W: wr_reg_WB: %x, alu__out_wb: %x, ctrl_we_WB: %x, mem_WB: %x", wr_reg_WB, alu__out_WB, ctrl_we_WB, mem_write_en_WB);
       //$display ("sys: %x, rt_data: %x", ctrl_Sys, rt_data);
       //$display ("sys_WB: %x, rt_data: %x", ctrl_Sys_WB, rt_data_WB);
       $display ("stall: %x, CDen: %x", stall, CDen);
       //$display ("Address: %h, Store: %h, Load:%h, en:%b", mem_addr, mem_data_in, mem_data_out, mem_write_en);
       //$display ("alu_in1: %d, alu_in2: %d, brcond: %b", alu_in1, alu_in2,brcond);
       $display ("branchTrue: %b, pcMuxSel: %b, pcMuxSelFinal: %b, brcond_EX: %b", branchTrue, pcMuxSel, pcMuxSelFinal, brcond_EX);
       $display ("j_target: %x, br_target: %x, pc_EX: %x, imm_EX: %x", j_target, br_target, pc_EX, imm_EX);
       $display ("jLink_en_WB: %x, wr_data: %x, wr_reg: %x", jLink_en_WB, wr_data, wr_reg);
       $display ("");
     end
   end
   // synthesis translate_on

   // Let Verilog-Mode pipe wires through for us.  This is another example
   // of Verilog-Mode's power -- undeclared nets get AUTOWIREd up when we
   // run 'make auto'.
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   wire [3:0]		alu__sel;		// From Decoder of mips_decode.v
   wire			ctrl_RI;		// From Decoder of mips_decode.v
   wire			ctrl_Sys;		// From Decoder of mips_decode.v
   wire			ctrl_we;		// From Decoder of mips_decode.v
   // End of automatics

   //Added control signals
   wire regdst;
   wire jLink_en;
   wire[2:0] brcond;
   wire branchTrue;
   wire [1:0] memtoreg;
   wire alusrc1;
   wire alusrc2;
   wire se;
   wire [3:0] mem_en; //memory write enable
   wire [2:0] load_sel; //selects the type of load for the loader to perform
   wire hi_en;
   wire lo_en;
   wire [1:0] store_sel; //selects the type of store for the storer to perform
   wire load_stall; //check if instruction is a load or MF to signal a stall

   // Generate control signals
   mips_decode Decoder(/*AUTOINST*/
		       // Outputs
		       .ctrl_we		(ctrl_we),
		       .ctrl_Sys	(ctrl_Sys),
		       .ctrl_RI		(ctrl_RI),
		       .alu__sel	(alu__sel[3:0]),
                       .regdst          (regdst),
                       .pcMuxSel        (pcMuxSel),
                       .jLink_en        (jLink_en),
                       .memtoreg        (memtoreg),
                       .alusrc1         (alusrc1),
                       .alusrc2         (alusrc2),
                       .se              (se),
                       .mem_write_en    (mem_en),
                       .hi_en           (hi_en),
                       .lo_en           (lo_en),
                       .load_sel        (load_sel),
                       .brcond          (brcond),
                       .store_sel       (store_sel),
                       .load_stall      (load_stall),
		       // Inputs
		       .dcd_op		(dcd_op[5:0]),
		       .dcd_funct2	(dcd_funct2[5:0]),
                       .dcd_rt          (dcd_rt));


 
   // Register File
   // Instantiate the register file from regfile.v here.
   // Don't forget to hookup the "halted" signal to trigger the register dump 

/************Main Added Code**************************************************/

   //New wirings
   wire [4:0] wr_reg; //input to write register
   wire [31:0] imm; //signed or unsigned immediate
   wire [31:0] wr_data; //data to write to register file
   wire [31:0] wr_dataMem; //intermediate data to write to register file
   wire [4:0] wr_regNum;//intermediate reg to write to 
   wire [4:0] rt_regNum;//rt register to read from

   wire [31:0] br_target; //branch target
   wire [31:0] j_target; //unconditional jump target
   wire [31:0] hi_out; //HI Register out
   wire [31:0] lo_out; //LO Register out
   wire [31:0] hi_in; //HI Register in
   wire [31:0] lo_in; //LO Register in
   wire [31:0] load_data; //loaded data
   wire [31:0] store_data; //data to store
   
   wire [31:0] alu_in1; //mux output of rs_data and rt_data
   wire [31:0] alu_in2; //mux output of rt_data and signed/unsigned imm to ALU
   wire [31:0] rs_fwd; //forwarded value of rs
   wire [31:0] rt_fwd; //forwarded value of rt
   wire [1:0] fwd_rs_sel, fwd_rt_sel; //select bits for forwarding rs and rt

   //Fetch (IF) stage for pc register
   wire IFen; //enable for IF stage
   wire [31:0] stallpc; //either PC or PC+4 depending on stall conditions
   mux2to1 stallMux(stallpc, pc, pc+4, IFen);

   //Decode (ID) stage registers and wirings
   wire IDen; //enable for decode stage
   wire [31:0] pc_ID;
   register pcID(pc_ID, pc, clk, IDen, rst_b);
   register irD(inst_ID, inst, clk, IDen, rst_b);

   //Execute (EX) stage registers
   wire EXen; //enable for execute stage
   wire [31:0] pc_EX;
   wire [31:0] rs_data_EX;
   wire [31:0] rt_data_EX;
   wire [31:0] imm_EX;
   wire [4:0] wr_reg_EX;
   wire [1:0] fwd_rs_sel_EX, fwd_rt_sel_EX;
   wire [25:0] dcd_target_EX;
   register pcEX(pc_EX, pc_ID, clk, EXen, rst_b);
   register rsEX(rs_data_EX, rs_data, clk, EXen, rst_b);
   register rtEX(rt_data_EX, rt_data, clk, EXen, rst_b);
   register iEX(imm_EX, imm, clk, EXen, rst_b);
   register #(5) wrEX(wr_reg_EX, wr_regNum, clk, EXen, rst_b);
   register #(2) fwdrsEX(fwd_rs_sel_EX, fwd_rs_sel, clk, EXen, rst_b);
   register #(2) fwdrtEX(fwd_rt_sel_EX, fwd_rt_sel, clk, EXen, rst_b);
   register #(26) targetEX(dcd_target_EX, dcd_target, clk, EXen, rst_b);

   wire ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX;
   wire alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, load_stall_EX; 
   wire [1:0] memtoreg_EX, pcMuxSel_EX, store_sel_EX;
   wire [3:0] alu__sel_EX, mem_write_en_EX;
   wire [2:0] load_sel_EX, brcond_EX;
   cntlRegister cntlEX(ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX,
                       alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, memtoreg_EX, pcMuxSel_EX,
                       alu__sel_EX, mem_write_en_EX, load_sel_EX, brcond_EX, store_sel_EX, load_stall_EX,
                       //Inputs
                       ctrl_we, ctrl_Sys, ctrl_RI, regdst, jLink_en, 
                       alusrc1, alusrc2, se, hi_en, lo_en, memtoreg, pcMuxSel,
                       alu__sel, mem_en, load_sel, brcond, store_sel,load_stall,
                       clk, EXen, rst_b);

   //Memory (MEM) stage registers
   wire MEMen; //enable for memory stage
   wire [31:0] pc_MEM;
   wire [31:0] alu__out_MEM;
   wire [31:0] rt_data_MEM;
   wire [31:0] imm_MEM;
   wire [4:0] wr_reg_MEM;
   assign MEMen = 1; //MEM stage never disabled
   register pcMEM(pc_MEM, pc_EX, clk, MEMen, rst_b);
   register aluMEM(alu__out_MEM, alu__out, clk, MEMen, rst_b);
   register rtMEM(rt_data_MEM, rt_fwd, clk, MEMen, rst_b);
   register iMEM(imm_MEM, imm_EX, clk, MEMen, rst_b);
   register #(5) wrMEM(wr_reg_MEM, wr_reg_EX, clk, MEMen, rst_b);

   wire ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM;
   wire alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, load_stall_MEM;
   wire [1:0] memtoreg_MEM, pcMuxSel_MEM, store_sel_MEM;
   wire [3:0] alu__sel_MEM, mem_write_en_MEM;
   wire [2:0] load_sel_MEM, brcond_MEM;
   cntlRegister cntlMEM(ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM,
                       alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, memtoreg_MEM, pcMuxSel_MEM,
                       alu__sel_MEM, mem_write_en_MEM, load_sel_MEM, brcond_MEM, store_sel_MEM,load_stall_MEM,
                       //Inputs
                       ctrl_we_EX, ctrl_Sys_EX, ctrl_RI_EX, regdst_EX, jLink_en_EX,
                       alusrc1_EX, alusrc2_EX, se_EX, hi_en_EX, lo_en_EX, memtoreg_EX, pcMuxSel_EX,
                       alu__sel_EX, mem_write_en_EX, load_sel_EX, brcond_EX, store_sel_EX,load_stall_EX,
                       clk, MEMen, rst_b);

   //Writeback stage registers
   wire WBen; //enable for WB stage
   wire [31:0] pc_WB;
   wire [31:0] HIout_WB, LOout_WB, load_data_WB, alu__out_WB;
   wire [31:0] rt_data_WB;
   wire [4:0] wr_reg_WB;
   assign WBen = 1; //WB stage never disabled
   register pcWB(pc_WB, pc_MEM, clk, WBen, rst_b);
   register MDRw(load_data_WB, load_data, clk, WBen, rst_b);
   register Aoutw(alu__out_WB, alu__out_MEM, clk, WBen, rst_b);
   register HIwb(HIout_WB, hi_out, clk, WBen, rst_b); //holds HI val in WB register (may need to have its own en)
   register LOwb(LOout_WB, lo_out, clk, WBen, rst_b); //holds LO val in WB register (may need to have its own en)
   register rtWB(rt_data_WB, rt_data_MEM, clk, WBen, rst_b);
   register #(5) wrWB(wr_reg_WB, wr_reg_MEM, clk, WBen, rst_b);

   wire ctrl_we_WB, ctrl_Sys_WB, ctrl_RI_WB, regdst_WB, jLink_en_WB;
   wire alusrc1_WB, alusrc2_WB, se_WB, hi_en_WB, lo_en_WB, load_stall_WB;
   wire [1:0] memtoreg_WB, pcMuxSel_WB, store_sel_WB;
   wire [3:0] alu__sel_WB, mem_write_en_WB;
   wire [2:0] load_sel_WB, brcond_WB;
   cntlRegister cntlWB(ctrl_we_WB, ctrl_Sys_WB, ctrl_RI_WB, regdst_WB, jLink_en_WB,
                       alusrc1_WB, alusrc2_WB, se_WB, hi_en_WB, lo_en_WB, memtoreg_WB, pcMuxSel_WB,
                       alu__sel_WB, mem_write_en_WB, load_sel_WB, brcond_WB, store_sel_WB,load_stall_WB,
                       //Inputs
                       ctrl_we_MEM, ctrl_Sys_MEM, ctrl_RI_MEM, regdst_MEM, jLink_en_MEM,
                       alusrc1_MEM, alusrc2_MEM, se_MEM, hi_en_MEM, lo_en_MEM, memtoreg_MEM, pcMuxSel_MEM,
                       alu__sel_MEM, mem_write_en_MEM, load_sel_MEM, brcond_MEM, store_sel_MEM, load_stall_MEM,
                       clk, WBen, rst_b);

   //check for RAW hazard and Stall
   
   wire stall, CDen, EXenStall;
   wire [2:0] CDAmt;
   stallDetector sD(wr_reg_EX,wr_reg_MEM,wr_reg_WB,rt_regNum,dcd_rs,
                    mem_write_en_EX, mem_write_en_MEM, mem_write_en,
                    ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB, regdst, stall,
                    load_stall_EX,
                    EXenStall, IDen, IFen, 
                    CDen, CDAmt);
   countdownReg cdsDReg(CDen, clk, rst_b,
                      CDAmt,
                      stall);

   //check for branch -> flush
   wire flush, CDFlushen, EXenFlush;
   wire [2:0] CDFlushAmt;
   flushMod fM(pcMuxSelFinal, flush,
               EXenFlush, CDFlushen,
               CDFlushAmt);
   countdownReg cdFlushReg(CDFlushen, clk, rst_b,
                           CDFlushAmt,
                           flush);

   assign EXen = (EXenFlush & EXenStall);

   //Register file
   regfile_forward RegFile(rs_data, rt_data, dcd_rs, rt_regNum, wr_reg, wr_data, ctrl_we_WB, clk, rst_b, halted);
   mux2to1 #(5) regDest(wr_regNum, dcd_rt, dcd_rd, regdst); //register to write to
   mux2to1 #(5) regRt(rt_regNum, dcd_rt, 5'd2, ctrl_Sys); //rt reg to read from (for syscalls)   

   //HI, LO registers
   register #(32,0) hiReg(hi_out, rs_fwd, clk, hi_en_EX, rst_b);
   register #(32,0) loReg(lo_out, rs_fwd, clk, lo_en_EX, rst_b);

   //Determines inputs to ALU
   mux2to1 aluSrc1(alu_in1, rs_fwd, rt_fwd, alusrc1_EX); //ALUSrc1
   mux2to1 aluSrc2(alu_in2, rt_fwd, imm_EX, alusrc2_EX); //ALUSrc2
   mux2to1 signext(imm, dcd_e_imm, dcd_se_imm, se); //Zero extend or sign extend immediate

   //rs and rt forwarding
   mux4to1 fwdrs(rs_fwd, rs_data_EX, alu__out_MEM, wr_dataMem, , fwd_rs_sel_EX);
   mux4to1 fwdrt(rt_fwd, rt_data_EX, alu__out_MEM, wr_dataMem, , fwd_rt_sel_EX);
   forwardData fwd(wr_reg_EX, wr_reg_MEM, wr_reg_WB, rt_regNum, dcd_rs,
                   mem_write_en_EX, mem_write_en_MEM, mem_write_en,
                   ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB,
                   fwd_rs_sel, fwd_rt_sel);

   //Wirings to memory module
   mux4to1 memToReg(wr_dataMem, alu__out_WB, load_data_WB, HIout_WB, LOout_WB, memtoreg_WB);
   //assign instr_addr = newpc[31:2]; //address of next instruction
   assign mem_addr = alu__out_MEM[31:2]; //memory address to read/write
   assign mem_data_in = store_data; //data to store

   //To read from / write to memory
   loader loader(load_data, imm_MEM, mem_data_out, load_sel_MEM, alu__out_MEM); //operates on data loaded from memory
   storer storer(store_data, mem_write_en, rt_data_MEM, store_sel_MEM, alu__out_MEM, mem_write_en_MEM); //operates on data to write to memory

   //Mux for next state PC
   mux4to1 pcMux(newpc, stallpc, br_target, rs_fwd, j_target, pcMuxSelFinal); //chooses next PC depending on jump or branch
   adder brtarget(br_target, pc_EX+4, (imm_EX << 2), 1'b0); //get branch target
   concat conc(j_target, pc_EX, dcd_target_EX); //get jump target
   pcSelector choosePcMuxSel(pcMuxSelFinal, pcMuxSel_EX, branchTrue); //chooses PC on whether branch condition is met


   //Set wr_data and wr_reg when there is a jump/branch with link
   mux2to1 dataToReg(wr_data, wr_dataMem, pc_WB+4, jLink_en_WB); 
   mux2to1 #(5)regNumber(wr_reg, wr_reg_WB, 5'd31, jLink_en_WB);

/*****************************************************************************/


   // Execute
   mips_ALU ALU(.alu__out(alu__out),
                .branchTrue(branchTrue), 
                .alu__op1(alu_in1),
                .alu__op2(alu_in2),
                .alu__sel(alu__sel_EX),
                .brcond(brcond_EX));
 
   // Miscellaneous stuff (Exceptions, syscalls, and halt)
   exception_unit EU(.exception_halt(exception_halt), .pc(pc), .rst_b(rst_b),
                     .clk(clk), .load_ex_regs(load_ex_regs),
                     .load_bva(load_bva), .load_bva_sel(load_bva_sel),
                     .cause(cause_code),
                     .IBE(inst_excpt),
                     .DBE(1'b0),
                     .RI(ctrl_RI),
                     .Ov(1'b0),
                     .BP(1'b0),
                     .AdEL_inst(pc[1:0]?1'b1:1'b0),
                     .AdEL_data(1'b0),
                     .AdES(1'b0),
                     .CpU(1'b0));

   assign r_v0 = rt_data_WB; // rt_data for syscall is data from $v0

   syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc), .clk(clk), .Sys(ctrl_Sys_WB),
                   .r_v0(r_v0), .rst_b(rst_b));
   assign        internal_halt = exception_halt | syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   register #(32, 0) EPCReg(epc, pc, clk, load_ex_regs, rst_b);
   register #(32, 0) CauseReg(cause,
                              {25'b0, cause_code, 2'b0}, 
                              clk, load_ex_regs, rst_b);
   register #(32, 0) BadVAddrReg(bad_v_addr, pc, clk, load_bva, rst_b);   

endmodule // mips_core


////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out (output) - Final result
//// branchTrue (output) - Whether the branch condition is met
//// brcond (input) - Set if instruction is a branch
//// in1 (input)  - Operand modified by the operation
//// in2 (input)  - Operand used (in arithmetic ops) to modify in1
//// sel (input)  - Selects which operation is to be performed
////
module mips_ALU(alu__out, branchTrue, alu__op1, alu__op2, alu__sel, brcond);

   output logic [31:0] alu__out;
   output logic branchTrue;
   input logic [31:0]  alu__op1, alu__op2;
   input logic [3:0]   alu__sel;
   input logic [2:0]   brcond;

   always_comb begin
    alu__out = 0;
    branchTrue = 0;
    case (alu__sel)
      `ALU_ADD:
        alu__out = alu__op1+alu__op2;
      `ALU_SUB:
        begin //check if branch condition is met
	  alu__out = alu__op1 - alu__op2;
          //$display("sub");
          case(brcond)
            `BR_BLTZ:
              begin
                if ($signed(alu__op1)<0)
                  branchTrue = 1'b1;
              end
            `BR_BGEZ:
              begin
                if ($signed(alu__op1)>=0)
                  branchTrue = 1'b1;
              end
            `BR_BEQ:
              begin
                if (alu__op1 == alu__op2)
                  branchTrue = 1'b1;
              end
            `BR_BNE:
              begin
                if (alu__op1!=alu__op2)
                  branchTrue = 1'b1;
              end
            `BR_BLEZ:
              begin
                if ($signed(alu__op1)<=$signed(0))
                  branchTrue = 1'b1;
              end
            `BR_BGTZ:
              begin
                //$display("alu__op1: %d, alu__op2:%d", alu__op1, alu__op2);
                if ($signed(alu__op1)>$signed(0))
                  branchTrue = 1'b1;
              end
            default:
              alu__out = alu__op1-alu__op2;
          endcase
        end
      `ALU_SLL:
        //shift by value in bits [10:6] of immediate
        alu__out = alu__op1<<{27'b0, alu__op2[10:6]};
      `ALU_SRL:
        alu__out = alu__op1>>{27'b0, alu__op2[10:6]};
      `ALU_SRA:
        //signed arithmetic shift
        alu__out = $signed($signed(alu__op1) >>> {27'b0, alu__op2[10:6]});
      `ALU_SLLV:
        alu__out = alu__op2<<{27'b0, alu__op1[4:0]};
      `ALU_SRLV:
        alu__out = alu__op2>>{27'b0, alu__op1[4:0]};
      `ALU_SRAV:
        alu__out = $signed($signed(alu__op2) >>> {27'b0, alu__op1[4:0]});
      `ALU_AND:
        alu__out = alu__op1 & alu__op2;
      `ALU_OR:
        alu__out = alu__op1 | alu__op2;
      `ALU_XOR:
        alu__out = alu__op1 ^ alu__op2;
      `ALU_NOR:
        alu__out = ~(alu__op1 | alu__op2);
      `ALU_SLT://signed compare
        alu__out = ($signed(alu__op1) < $signed(alu__op2)) ? 32'b1 : 32'b0;
      `ALU_SLTU://signed compare
        alu__out = (alu__op1 < alu__op2) ? 32'b1 : 32'b0;

    endcase

   end
   //adder AdderUnit(alu__out, alu__op1, alu__op2, alu__sel[0]);

endmodule

//// resetregister: A register which may be reset to an arbirary value and is set to 0 if disabled.
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module resetregister(q, d, clk, enable, clear, rst_b);

   parameter
            width = 32,
            reset_value = 0;

   output [(width-1):0] q;
   reg [(width-1):0]    q;
   input [(width-1):0]  d;
   input                 clk, enable, clear, rst_b;

   always @(posedge clk or negedge rst_b)
     if (~rst_b)
       q <= reset_value;
     else if (clear)
       q <= 32'b0;
     else if (enable)
       q <= d;

endmodule // register


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
         pcMuxSel <= 2'b0;
         alu__sel <= alu__sel_in;
         mem_write_en <= 1'b0;
         load_sel <= load_sel_in;
         brcond <= brcond_in;
         store_sel <= store_sel_in;
         load_stall <=load_stall_in;
       end

endmodule

////
//// forwardData: module for forwarding data to prevent RAW hazards
////
//// wr_reg_EX, wr_reg_MEM, wr_reg_WB (inputs) - are reg numbers at different stages
//// dcd_rs, dcd_rt (inputs) - the registers the next instruction is reading
//// mem_write_en_EX, mem_write_en_MEM, mem_write_en (inputs) - the memory write enable bits at different stages
//// ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB (inputs) - register write enable bits at different stages
//// rsfwd, rtfwd (output) - selects what data to forward to rs and rt data outputs
////

module forwardData(
  input logic [4:0] wr_reg_EX, wr_reg_MEM, wr_reg_WB, dcd_rt, dcd_rs,
  input logic [3:0] mem_write_en_EX, mem_write_en_MEM, mem_write_en,
  input logic ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB,
  output logic [1:0] rsfwd, rtfwd);

  always_comb begin
    rsfwd = 2'b0;
    rtfwd = 2'b0;
    if (ctrl_we_MEM != 0) begin
      if ((dcd_rs != 0) && (dcd_rs == wr_reg_MEM)) begin
        rsfwd = 2'b10;
      end
      if ((dcd_rt != 0) && (dcd_rt == wr_reg_MEM)) begin
        rtfwd = 2'b10;
      end
    end
    if (ctrl_we_EX != 0) begin
      if ((dcd_rs != 0) && (dcd_rs == wr_reg_EX)) begin
        rsfwd = 2'b01;
      end
      if ((dcd_rt != 0) && (dcd_rt == wr_reg_EX)) begin
        rtfwd = 2'b01;
      end
    end

  end

endmodule


////
//// flushMod: module for flushing if branch
////
//// pcMuxSelFinal (input) - determines if there is a branch
//// flush (input) - signal that indicates an instruction is currently being stalled
//// EXen (output) - register enable bits at the EX stage
//// CDFlushen  (output) - enables countdown register
//// CDAmt (output) - number of clock cycles to stall
////
module flushMod(
  input logic [1:0] pcMuxSelFinal,
  input logic flush,
  output logic EXen, CDFlushen,
  output logic [2:0] CDFlushAmt);
  always_comb begin

    EXen = 1'b1;
    CDFlushAmt = 3'b0;
    CDFlushen = 1'b0;
    if (flush == 1'b1) begin
      $display("flushmod1");
      EXen = 1'b0;
    end
    else if(flush ==1'b0 && pcMuxSelFinal != 2'b0) begin
      $display("flushmod2");
      CDFlushen = 1'b1;
      EXen = 1'b0;
      CDFlushAmt = 1'd1;
    end
  end
endmodule

////
//// stallDetector: module for enabling stalls if RAW hazard
////
//// wr_reg_EX, wr_reg_MEM, wr_reg_WB (inputs) - are reg numbers at different stages
//// dcd_rs, dcd_rt (inputs) - the registers the next instruction is reading
//// mem_write_en_EX, mem_write_en_MEM, mem_write_en (inputs) - the memory write enable bits at different stages
//// ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB (inputs) - register write enable bits at different stages
//// stall (input) - signal that indicates an instruction is currently being stalled
//// IFen, IDen, EXen, (output) - register enable bits at the IF, ID, and EX stages
//// CDAmt (output) - number of clock cycles to stall
////
module stallDetector(
  input logic [4:0] wr_reg_EX, wr_reg_MEM, wr_reg_WB, dcd_rt, dcd_rs,
  input logic [3:0] mem_write_en_EX, mem_write_en_MEM, mem_write_en,
  input logic ctrl_we_EX, ctrl_we_MEM, ctrl_we_WB, regdst, stall,
  input logic load_stall_EX,
  output logic EXen, IDen, IFen, CDen,
  output logic [2:0] CDAmt);
  
  always_comb begin
    IFen = 1'b1;
    IDen = 1'b1;
    EXen = 1'b1;
    CDen = 1'b0;
    CDAmt = 3'b0;
    if (stall == 1'b1) begin
      IFen = 1'b0;
      IDen = 1'b0;
      EXen = 1'b0;
    end
    else if(stall == 1'b0 && load_stall_EX == 1'b1) begin
      if ((ctrl_we_EX != 0 || mem_write_en_EX != 0) && (((regdst == 1) && (dcd_rt != 0) && (dcd_rt == wr_reg_EX)) || ((regdst == 0) && (dcd_rs != 0) && (dcd_rs == wr_reg_EX)))) begin
        $display("we_EX: %x, mem_EX: %x, regdst: %x, dcd_rt: %x, dcd_rs: %x, wr_reg_EX: %x", ctrl_we_EX, mem_write_en_EX, regdst, dcd_rt, dcd_rs, wr_reg_EX);
        CDen = 1'b1;
        CDAmt = 3'd0;
        IFen = 1'b0;
        IDen = 1'b0;
        EXen = 1'b0;
      end 
    end
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
//// mux2to1
////
//// out (output) - data chosen to be outputted
//// in0 (input)  - data lines
//// in1 (input)  - data lines
//// sel (input)  - selects which data to output
////
module mux2to1 #(parameter width = 32) (
      output logic [width - 1:0] out,
      input logic [width - 1:0] in0, in1, 
      input logic sel);
    
    assign out = sel ? in1 : in0;

endmodule


////
//// mux4to1
////
//// out (output) - data chosen to be outputted
//// in0 (input)  - data lines
//// in1 (input)  - data lines
//// in2 (input)  - data lines
//// in3 (input)  - data lines
//// sel (input)  - selects which data to output
////
module mux4to1 #(parameter width = 32) (
      output logic [width - 1:0] out,
      input logic [width - 1:0] in0, in1, in2, in3,
      input logic [1:0] sel);

    assign out = sel[1] ? (sel[0] ? in3 : in2) : (sel[0] ? in1 : in0);

endmodule

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

////
//// loader: operates on data for load instructions
////
//// load_data (output) - data to load into registers
//// dcd_imm   (input)  - immediate (for LUI)
//// mem_data  (input)  - data read from memory
//// load_sel  (input)  - selects what to output
//// offset    (input)  - byte offset for writing to memory
////
module loader (
      output logic [31:0] load_data,
      input logic [15:0] dcd_imm,
      input logic [31:0] mem_data,
      input logic [2:0] load_sel,
      input logic [31:0] offset);

    //shift the data by the offset number of bytes
    logic [31:0] data;
    assign data = (mem_data >> ((offset & 32'h3) * 8));

    always_comb begin
      case(load_sel)
        `LOAD_LUI: //load upper immediate
          load_data = {dcd_imm, 16'b0};
        `LOAD_LB: //load top portion of shifted data
          load_data = {{24{data[7]}}, data[7:0] };
        `LOAD_LH:
          load_data = {{16{data[15]}}, data[15:0]};
        `LOAD_LW:
          load_data = mem_data;
        `LOAD_LBU:
          load_data = {24'b0, data[7:0]};
        `LOAD_LHU:
          load_data = {16'b0, data[15:0]};
        default:
          load_data = 32'hxxxx;
      endcase
    end
endmodule

////
//// storer: operates on data for store instructions
////
//// store_data (output) - data to load into registers
//// mem_en (output) - shifted memory write enable bits
//// rt_data   (input)  - data from rt register
//// store_sel  (input)  - selects which store operation to perform
//// offset    (input)  - byte offset
//// mem_write_en (input) - memory enable bits from control module
////
module storer (
      output logic [31:0] store_data,
      output logic [3:0] mem_write_en,
      input logic [31:0] rt_data,
      input logic [1:0] store_sel,
      input logic [31:0] offset,
      input logic [3:0] mem_en);

    always_comb begin
      case(store_sel)
        `ST_SB: //place data in top bits of word and shift right by offset number of bytes
          begin
            store_data = {24'b0, rt_data[7:0]} << ((offset & 32'h3) * 8);
            mem_write_en = mem_en >> (offset & 32'h3);
          end
        `ST_SH:
          begin
            store_data = {16'b0, rt_data[15:0]} << ((offset & 32'h3) * 8);
            mem_write_en = mem_en >> (offset & 32'h3);
          end
        `ST_SW:
          begin
            store_data = rt_data;
            mem_write_en = mem_en;
          end
        default:
          begin
            store_data = 32'hxxxx;
            mem_write_en = 4'b0000;
          end
      endcase
    end
endmodule


// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
