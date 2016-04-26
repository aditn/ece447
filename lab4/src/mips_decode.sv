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

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// mips_decode: Decode MIPS instructions
//// Refer to MIPS Instruction Set to see different Instruction formats
////
//// op      (input)  - Instruction opcode
//// funct2  (input)  - Instruction minor opcode
//// rt      (input)  - Instruction minor opcode
//// alu__sel(output) - Selects the ALU function
//// we      (output) - Write to the register file
//// Sys     (output) - System call exception
//// RI      (output) - Reserved instruction exception
//// regdst  (output) - Selects the destination register (Rt or Rd)
//// pcMuxSel(output) - Chooses next PC in pcMux (00: pc+4;
////                    01: conditional branching;
////                    10: picks rsdata for addr in register
////                    11: jump to 26 bit imm)
//// memtoreg(output) - Selects either ALU or memory to write to a register
//// alusrc1 (output) - Selects the first input to the ALU (rs_data or rt_data)
//// alusrc2 (output) - Selects the second input to the ALU (register data or immediate)
//// se      (output) - Selects whether to sign extend the immediate
//// mem_write_en (output) - Which portion of a word to write to memory
//// hi_en (output) - Enables writing to HI register
//// lo_en (output) - Enables writing to LO register
//// load_sel (output) - Selects which load operation for loader to perform
//// brcond  (output) - Branch condition
//// store_sel (output) - Selects which store operation for storer to perform
//// load_stall (output) - Signals that the instruction is a load or MF type instruction
////

module mips_decode(/*AUTOARG*/
   // Outputs
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, regdst, pcMuxSel, jLink_en,
   memtoreg, alusrc1, alusrc2, se, mem_write_en, hi_en,lo_en,
   load_sel, brcond, store_sel,
   load_stall,

   // Inputs
   dcd_op, dcd_funct2, dcd_rt
   );

   input       [5:0] dcd_op, dcd_funct2;
   input       [4:0] dcd_rt;
   output reg        ctrl_we, ctrl_Sys, ctrl_RI, regdst, jLink_en, alusrc1, alusrc2, se, hi_en, lo_en;
   output reg  [1:0] memtoreg, pcMuxSel;
   output reg  [3:0] alu__sel, mem_write_en;
   output reg  [2:0] load_sel,brcond;
   output reg  [1:0] store_sel;
   output reg        load_stall;
 

   always_comb begin
     alu__sel = 4'hx; //no speciic alu operation chosen yet
     ctrl_we = 1'b1; //is reg write
     ctrl_Sys = 1'b0;
     ctrl_RI = 1'b0;

     regdst = 1'b0; //destination reg is Rt
     pcMuxSel = 2'b00; //not jump
     jLink_en = 1'b0; //not linking

     memtoreg = 2'b00; //write to reg from ALU, not mem
     alusrc1 = 1'b0; //source is rs_data
     alusrc2 = 1'b0; //source is register

     se = 1'b0; //unsigned

     mem_write_en = 4'b0; //no mem write
     load_sel = `NO_LOAD;
     store_sel = 2'bx;
     load_stall = 1'b0; //not currently performing a load instruction
     hi_en = 1'b0; //HI reg not enabled
     lo_en = 1'b0; //LO reg not enabled


     brcond = 3'b111; 

     case(dcd_op) // Main opcodes (op field)
       `OP_OTHER0: // Secondary opcodes (funct2 field; OP_OTHER0)
         begin
           regdst = 1'b1; //destination is Rd
         case(dcd_funct2)
           `OP0_SLL:
             begin
               alu__sel = `ALU_SLL;
               alusrc1 = 1'b1;
               alusrc2 = 1'b1;
             end
          `OP0_SRL:
             begin
               alu__sel = `ALU_SRL;
               alusrc1 = 1'b1;
               alusrc2 = 1'b1;
             end
           `OP0_SRA:
             begin
               alu__sel = `ALU_SRA;
               alusrc1 = 1'b1;
               alusrc2 = 1'b1;
             end
           `OP0_SLLV:
             alu__sel = `ALU_SLLV;
           `OP0_SRLV:
             alu__sel = `ALU_SRLV;
           `OP0_SRAV:
             alu__sel = `ALU_SRAV;
           `OP0_JR: //jump to addr in reg
             begin
               ctrl_we = 1'b0;
               pcMuxSel = 2'b10;
             end
           `OP0_JALR:
             begin
             //need to get PC+4 into $ra
               pcMuxSel = 2'b10;
               jLink_en = 1'b1;
               ctrl_we = 1'b1;
             end
           `OP0_SYSCALL:
             ctrl_Sys = 1'b1;
           `OP0_MFHI: //read from HI reg
             begin
               memtoreg=2'b10;
               load_stall=1'b1;
             end
           `OP0_MTHI: //write to HI reg
             hi_en = 1'b1;
           `OP0_MFLO: //read from LO reg
             begin
               memtoreg=2'b11;
               load_stall=1'b1;
             end
           `OP0_MTLO: //write to LO reg
             lo_en = 1'b1;
           `OP0_ADD: 
             alu__sel = `ALU_ADD;
           `OP0_ADDU:
             alu__sel = `ALU_ADD; //same as add
           `OP0_SUB:
             alu__sel = `ALU_SUB;
           `OP0_SUBU:
             alu__sel = `ALU_SUB; //same as sub
           `OP0_AND:
             alu__sel = `ALU_AND;
           `OP0_OR:
             alu__sel = `ALU_OR;
           `OP0_XOR:
             alu__sel = `ALU_XOR;
           `OP0_NOR:
             alu__sel = `ALU_NOR;
           `OP0_SLT:
             alu__sel = `ALU_SLT;
           `OP0_SLTU:
             alu__sel = `ALU_SLTU;
           default:
             ctrl_RI = 1'b1;
         endcase //funct2
         end //OP_OTHER0

       `OP_OTHER1: // Secondary opcodes (rt field; OP_OTHER1)
         case(dcd_rt)
            //conditional branching are found in the ALU
           `OP1_BLTZ:
             begin
               alu__sel = `ALU_SUB;
               ctrl_we = 1'b0;
               pcMuxSel = 2'b01;
               brcond = `BR_BLTZ;
             end
           `OP1_BGEZ:
             begin
               alu__sel = `ALU_SUB;
               ctrl_we = 1'b0;
               pcMuxSel = 2'b01;
               brcond = `BR_BGEZ;
             end
           `OP1_BLTZAL:
             begin
               alu__sel = `ALU_SUB;
               pcMuxSel = 2'b01;
               jLink_en = 1'b1;
               brcond = `BR_BLTZ;
             end
           `OP1_BGEZAL:
             begin
               alu__sel = `ALU_SUB; //for brcond
               pcMuxSel = 2'b01;
               jLink_en = 1'b1;
               brcond = `BR_BGEZ;
             end
           default:
             ctrl_RI = 1'b1;
         endcase //dcd_rt
       
       `OP_J:
         begin
           ctrl_we = 1'b0;
           pcMuxSel = 2'b11;
         end

       `OP_JAL:
         begin
           ctrl_we = 1'b1; //need to write to $ra
           pcMuxSel = 2'b11;
           jLink_en = 1'b1;
         end
       `OP_BEQ:
         begin
           alu__sel = `ALU_SUB; //for brcond
           ctrl_we = 1'b0;
           pcMuxSel = 2'b01;
           brcond = `BR_BEQ;
           se = 1'b1;
         end
       `OP_BNE:
         begin
           alu__sel = `ALU_SUB; //for brcond
           ctrl_we = 1'b0;
           pcMuxSel = 2'b01;
           brcond = `BR_BNE;
           se = 1'b1;
         end
       `OP_BLEZ:
         begin
           alu__sel = `ALU_SUB; //for brcond
           ctrl_we = 1'b0;
           pcMuxSel = 2'b01;
           brcond = `BR_BLEZ;
           se = 1'b1;
         end
       `OP_BGTZ:
         begin
           alu__sel = `ALU_SUB; //for brcond
           ctrl_we = 1'b0;
           pcMuxSel = 2'b01;
           brcond = `BR_BGTZ;
           se = 1'b1;
         end
       `OP_ADDI:
         begin
           alu__sel = `ALU_ADD;
           alusrc2 = 1'b1;
           se = 1'b1;
         end
       `OP_ADDIU:
         begin
           alu__sel = `ALU_ADD;
           alusrc2 = 1'b1;
           se = 1'b1; //need to sign extend to read inputs right
         end
       `OP_SLTI:
         begin
           alu__sel = `ALU_SLT;
           alusrc2 = 1'b1;
           se = 1'b1;
         end
       `OP_SLTIU:
         begin
           alu__sel = `ALU_SLTU;
           alusrc2 = 1'b1;
         end
       `OP_ANDI:
         begin
           alu__sel = `ALU_AND;
           alusrc2 = 1'b1;
         end
       `OP_ORI:
         begin
           alu__sel = `ALU_OR;
           alusrc2 = 1'b1;
         end
       `OP_XORI:
         begin
           alu__sel = `ALU_XOR;
           alusrc2 = 1'b1;
         end
       `OP_LUI:
         begin
           memtoreg = 2'b01;
           load_stall = 1'b1;
           load_sel = `LOAD_LUI;
         end
       `OP_LB:
         begin
           load_stall = 1'b1;
           alu__sel = `ALU_ADD;
           se = 1'b1;
           alusrc2 = 1'b1;
           memtoreg = 1'b1;
           load_sel = `LOAD_LB;
         end
       `OP_LH:
         begin
           load_stall = 1'b1;
           alu__sel = `ALU_ADD;
           se = 1'b1;
           alusrc2 = 1'b1;
           memtoreg = 1'b1;
           load_sel = `LOAD_LH;
         end
       `OP_LW:
         begin
           load_stall = 1'b1;
           alu__sel = `ALU_ADD;
           se = 1'b1;
           alusrc2 = 1'b1;
           memtoreg = 1'b1;
           load_sel = `LOAD_LW;
         end
       `OP_LBU:
         begin
           load_stall = 1'b1;
           alu__sel = `ALU_ADD;
           se = 1'b1;
           alusrc2 = 1'b1;
           memtoreg = 1'b1;
           load_sel = `LOAD_LBU;
         end
       `OP_LHU:
         begin
           load_stall = 1'b1;
           alu__sel = `ALU_ADD;
           se = 1'b1;
           alusrc2 = 1'b1;
           memtoreg = 1'b1;
           load_sel = `LOAD_LHU;
         end
       `OP_SB:
         begin
           ctrl_we = 1'b0;
           alu__sel = `ALU_ADD;
           alusrc2 = 1'b1;
           se = 1'b1;
           regdst = 1'b1;
           mem_write_en = 4'b1000;
           store_sel = `ST_SB;
         end
       `OP_SH:
         begin
           ctrl_we = 1'b0;
           alu__sel = `ALU_ADD;
           alusrc2 = 1'b1;
           se = 1'b1;
           regdst = 1'b1;
           mem_write_en = 4'b1100;
           store_sel = `ST_SH;
         end
       `OP_SW:
         begin
           ctrl_we = 1'b0;
           alu__sel = `ALU_ADD;
           alusrc2 = 1'b1;
           se = 1'b1;
           regdst = 1'b1;
           mem_write_en = 4'b1111;
           store_sel = `ST_SW;
         end
       default:
         begin
            ctrl_RI = 1'b1;
         end
     endcase // case(op)
   end

endmodule
