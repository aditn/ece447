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

   // Core Interface
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
   assign        dcd_op = inst[31:26];    // Opcode
   assign        dcd_rs = inst[25:21];    // rs field
   assign        dcd_rt = inst[20:16];    // rt field
   assign        dcd_rd = inst[15:11];    // rd field
   assign        dcd_shamt = inst[10:6];  // Shift amount
   assign        dcd_bczft = inst[16];    // bczt or bczf?
   assign        dcd_funct1 = inst[4:0];  // Coprocessor 0 function field
   assign        dcd_funct2 = inst[5:0];  // funct field; secondary opcode
   assign        dcd_offset = inst[15:0]; // offset field
        // Sign-extended offset for branches
   assign        dcd_se_offset = { {14{dcd_offset[15]}}, dcd_offset, 2'b00 };
        // Sign-extended offset for load/store
   assign        dcd_se_mem_offset = { {16{dcd_offset[15]}}, dcd_offset };
   assign        dcd_imm = inst[15:0];        // immediate field
   assign        dcd_e_imm = { 16'h0, dcd_imm };  // zero-extended immediate
        // Sign-extended immediate
   assign        dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
   assign        dcd_target = inst[25:0];     // target field
   assign        dcd_code = inst[25:6];       // Breakpoint code

   // synthesis translate_off
   /*always @(posedge clk) begin
     // useful for debugging, you will want to comment this out for long programs
     if (rst_b) begin
      
       $display ( "=== Simulation Cycle %d ===", $time );
       $display ( "[pc=%x, inst=%x] [op=%x, rs=%d, rt=%d, rd=%d, imm=%x, f2=%x] [reset=%d, halted=%d]",
                   pc, inst, dcd_op, dcd_rs, dcd_rt, dcd_rd, dcd_imm, dcd_funct2, ~rst_b, halted);
       $display ("Store address: %d, %d, Store word: %d, ALUOUT: %d, en: %d", rt_data, mem_addr, mem_data_in, alu__out, mem_write_en);
       $display ("HI: %x, LO: %x, pcMuxSel: %d, nextpc: %x, nextnextpc:%x", hi_out, lo_out,pcMuxSel,nextpc,nextnextpc);
       $display ("jLink_en:%d, wr_reg: %d, wr_data: %x",jLink_en, wr_reg, wr_data);
       $display ("Address: %h, Store: %h, Load:%h, en:%b", mem_addr, mem_data_in, mem_data_out, mem_write_en);
       $display ("alu_in1: %d, alu_in2: %d, brcond: %b", alu_in1, alu_in2,brcond);
       $display ("branchTrue: %b, pcMuxSel: %b, pcMuxSelFinal: %b", branchTrue, pcMuxSel, pcMuxSelFinal);
       $display ("br_target: %x", br_target);
       $display ("");
       $display ("cyclesCount:%d", cyclesCount);
       $display ("CFtaken: %d", cfTaken);
     end
   end*/
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

   //Control bits
   wire regdst;
   
   wire jLink_en;
   wire[2:0] brcond;
   wire branchTrue;

   wire [1:0] memtoreg;
   wire alusrc1;
   wire alusrc2;
   wire se;
   wire [2:0] load_sel;
   wire hi_en;
   wire lo_en;
   wire [1:0] store_sel;

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
                       .mem_write_en    (),
                       .hi_en           (hi_en),
                       .lo_en           (lo_en),
                       .load_sel        (load_sel),
                       .brcond          (brcond),
                       .store_sel       (store_sel),
		       // Inputs
		       .dcd_op		(dcd_op[5:0]),
		       .dcd_funct2	(dcd_funct2[5:0]),
           .dcd_rt (dcd_rt));


 
   // Register File
   // Instantiate the register file from regfile.v here.
   // Don't forget to hookup the "halted" signal to trigger the register dump 
 
   wire [31:0] alu_in1; // mux output of rs_data and rt_data
   wire [31:0] alu_in2; // mux output of rt_data and signed/unsigned imm to ALU

   // Execute
   mips_ALU ALU(.alu__out(alu__out),
                .branchTrue(branchTrue), 
                .alu__op1(alu_in1),
                .alu__op2(alu_in2),
                .alu__sel(alu__sel),
                .brcond(brcond));
 
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

   assign r_v0 = 32'h0a; // Good enough for now. To support syscall for real,
                         // you should read the syscall
                         // argument from $v0 of the register file 

   syscall_unit SU(.syscall_halt(syscall_halt), .pc(pc), .clk(clk), .Sys(ctrl_Sys),
                   .r_v0(r_v0), .rst_b(rst_b));
   assign        internal_halt = exception_halt | syscall_halt;
   register #(1, 0) Halt(halted, internal_halt, clk, 1'b1, rst_b);
   register #(32, 0) EPCReg(epc, pc, clk, load_ex_regs, rst_b);
   register #(32, 0) CauseReg(cause,
                              {25'b0, cause_code, 2'b0}, 
                              clk, load_ex_regs, rst_b);
   register #(32, 0) BadVAddrReg(bad_v_addr, pc, clk, load_bva, rst_b);

   //New wirings
   wire [4:0] wr_reg; //input to write register
   wire [31:0] imm; //signed or unsigned immediate
   wire [31:0] wr_data; //data to write to register file
   wire [31:0] wr_dataMem; //intermediate data to write to register file
   wire [31:0] wr_regNum;//intermediate reg to write to 

   wire [31:0] br_target; //branch target
   wire [31:0] j_target; //unconditional jump target
   wire [31:0] hi_out; //HI Register out
   wire [31:0] lo_out; //LO Register out
   wire [31:0] hi_in; //HI Register in
   wire [31:0] lo_in; //LO Register in
   wire [31:0] load_data;
   wire [31:0] store_data;

   //Register file
   regfile RegFile(rs_data, rt_data, dcd_rs, dcd_rt, wr_reg, wr_data, ctrl_we, clk, rst_b, halted);
   mux2to1 #(5) regDest(wr_regNum, dcd_rt, dcd_rd, regdst); //register to write to
   
   //HI, LO registers
   register #(32,0) hiReg(hi_out, hi_in, clk, hi_en, rst_b);
   register #(32,0) loReg(lo_out, lo_in, clk, lo_en, rst_b);
   assign hi_in = rs_data;
   assign lo_in = rs_data;

   //Determines inputs to ALU
   mux2to1 aluSrc1(alu_in1, rs_data, rt_data, alusrc1); //ALUSrc1
   mux2to1 aluSrc2(alu_in2, rt_data, imm, alusrc2); //ALUSrc2
   mux2to1 signext(imm, dcd_e_imm, dcd_se_imm, se); //Zero extend or sign extend immediate

   //Wirings to memory module
   mux4to1 memToReg(wr_dataMem, alu__out, load_data, hi_out, lo_out, memtoreg);
   assign instr_addr = newpc[31:2]; //address of next instruction
   assign mem_addr = alu__out[31:2]; //memory address to read/write
   assign mem_data_in = store_data; //data to store

   //To read from / write to memory
   loader loader(load_data, dcd_imm, mem_data_out, load_sel, alu__out); //operates on data loaded from memory
   storer storer(store_data, mem_write_en, rt_data, store_sel, alu__out); //operates on data to write to memory

   //Mux for next state PC
   wire [31:0] pcPlus4;
   carry_select cs_1(pc,32'd4,1'b0,pcPlus4,);
   carry_select cs_2(pcPlus4,(imm << 2),1'b0,br_target,);
   mux4to1 pcMux(newpc, pcPlus4, br_target, rs_data, j_target, pcMuxSelFinal); //chooses next PC depending on jump or branch
   //adder brtarget(br_target, pc + 4, (imm << 2), 1'b0); //get branch target
   concat conc(j_target, pc, dcd_target); //get jump target
   muxSpecial choosePcMuxSel(pcMuxSelFinal,pcMuxSel,branchTrue); //chooses PC on whether branch condition is met

   //Set wr_data and wr_reg when there is a jump/branch with link
   mux2to1 dataToReg(wr_data, wr_dataMem, pcPlus4, jLink_en); 
   mux2to1 #(31)regNumber(wr_reg, wr_regNum, 5'd31, jLink_en);


   /*Cycle Counter*/
   /*wire [31:0] cyclesCount;
   wire [31:0] cfTaken;
   counter cycles(cyclesCount, 1'b1, clk, rst_b);
   counter cycles2(cfTaken, pcMuxSelFinal!=2'b00, clk, rst_b);*/

endmodule // mips_core


////
//// counter: keep track of counts
////
//// en (input)     - enable bit for count
//// clk (input)    - Clock (positive edge-sensitive)
//// reset (input)  - System reset
//// count (output) - current count
////
module counter #(parameter reset_value = 32'd0) (
  output logic [31:0] count,
  input logic en, clk, rst_b);

  always_ff @(posedge clk or negedge rst_b) begin
    if (~rst_b) begin
      count <= reset_value;
    end
    else if (en) begin
      count <= count + 1;
    end
  end

endmodule



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
/*module mips_ALU(alu__out, branchTrue, alu__op1, alu__op2, alu__sel, brcond);

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
                $display("alu__op1: %d, alu__op2:%d", alu__op1, alu__op2);
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

endmodule*/

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
module muxSpecial #(parameter width = 2) (
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
//// mem_write_en (output) - memory write enable bits
//// rt_data   (input)  - data from rt register
//// store_sel  (input)  - selects which store operation to perform
//// offset    (input)  - byte offset
////
module storer (
      output logic [31:0] store_data,
      output logic [3:0] mem_write_en,
      input logic [31:0] rt_data,
      input logic [1:0] store_sel,
      input logic [31:0] offset);

    always_comb begin
      case(store_sel)
        `ST_SB: //place data in top bits of word and shift right by offset number of bytes
          begin
            store_data = {24'b0, rt_data[7:0]} << ((offset & 32'h3) * 8);
            mem_write_en = 4'b1000 >> (offset & 32'h3);
          end
        `ST_SH:
          begin
            store_data = {16'b0, rt_data[15:0]} << ((offset & 32'h3) * 8);
            mem_write_en = 4'b1100 >> (offset & 32'h3);
          end
        `ST_SW:
          begin
            store_data = rt_data;
            mem_write_en = 4'b1111;
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
