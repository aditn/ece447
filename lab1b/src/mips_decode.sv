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
////
//// op      (input)  - Instruction opcode
//// funct2  (input)  - Instruction minor opcode
//// rt      (input)  - Instruction minor opcode
//// alu_sel (output) - Selects the ALU function
//// we      (output) - Write to the register file
//// Sys     (output) - System call exception
//// RI      (output) - Reserved instruction exception
////
module mips_decode(/*AUTOARG*/
   // Outputs
   ctrl_we, ctrl_Sys, ctrl_RI, alu__sel, regdst, jmp, br, memtoreg, aluop, alusrc, se, mem_write_en,
   // Inputs
   dcd_op, dcd_funct2, dcd_rt
   );

   input       [5:0] dcd_op, dcd_funct2;
   input       [4:0] dcd_rt;
   output reg        ctrl_we, ctrl_Sys, ctrl_RI, regdst, jmp, br, memtoreg, aluop, alusrc, se;
   output reg  [3:0] alu__sel, mem_write_en;
 

   always_comb begin
     alu__sel = 4'hx;
     ctrl_we = 1'b1; //is reg write
     ctrl_Sys = 1'b0;
     ctrl_RI = 1'b0;

     regdst = 1'b0;
     jmp = 1'b0;
     br = 1'b0;
     memtoreg = 1'b0;
     aluop = 1'b0;
     alusrc = 1'b0; //source is register
     se = 1'b0;
     mem_write_en = 4'b0;
     case(dcd_op) // Main opcodes (op field)
       `OP_OTHER0: // Secondary opcodes (funct2 field; OP_OTHER0)
         begin
           regdst = 1'b1; //destination is Rd
           aluop = 1'b1; //is an ALU op
         case(dcd_funct2)
           `OP0_SLL:
             begin
               alu__sel = `ALU_SLL;
               alusrc = 1'b1;
             end
          `OP0_SRL:
             begin
               alu__sel = `ALU_SRL;
               alusrc = 1'b1;
             end
           `OP0_SRA:
             begin
               alu__sel = `ALU_SRA;
               alusrc = 1'b1;
             end
           `OP0_SLLV:
             alu__sel = `ALU_SLLV;
           `OP0_SRLV:
             alu__sel = `ALU_SRLV;
           `OP0_SRAV:
             alu__sel = `ALU_SRAV;
           `OP0_JR:
             begin
               ctrl_we = 1'b0;
               jmp = 1'b1;
               aluop = 1'b0;
             end
           `OP0_JALR:
             begin
             //need to get PC+4 into $ra
               jmp = 1'b1;
               aluop = 1'b0;
             end
           `OP0_SYSCALL:
             ctrl_Sys = 1'b1;
           //`OP0_MFHI:
           //`OP0_MTHI:
           //`OP0_MFLO:
           //`OP0_MTLO:
           `OP0_ADD: //how does this become 
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
             alu__sel = `ALU_SLT;
           default:
             ctrl_RI = 1'b1;
         endcase //funct2
         end //OP_OTHER0

       `OP_OTHER1: // Secondary opcodes (rt field; OP_OTHER1)
         case(dcd_rt)
           `OP1_BLTZ:
             begin
               alu__sel = `ALU_SLT; // for brcond
               ctrl_we = 1'b0;
               br = 1'b1;
               aluop = 1'b1;
             end
           `OP1_BGEZ:
             begin
               alu__sel = `ALU_SUB; //for brcond
               ctrl_we = 1'b0;
               br = 1'b1;
               aluop = 1'b1;
             end
           `OP1_BLTZAL:
             begin
               alu__sel = `ALU_SLT;
               br = 1'b1;
               aluop = 1'b1;
               //write PC+4 to $ra
             end
           `OP1_BGEZAL:
             begin
               alu__sel = `ALU_SUB; //for brcond
               br = 1'b1;
               aluop = 1'b1;
             end
           default:
             ctrl_RI = 1'b1;
         endcase //dcd_rt
         /*
       `OP_J:
       `OP_JAL:
       `OP_BEQ:
       `OP_BNE:
       `OP_BLEZ:
       `OP_BGTZ:
       `OP_ADDI:*/
       `OP_ADDIU:
         begin
            alu__sel = `ALU_ADD;
            ctrl_we = 1'b1;
         end/*
       `OP_SLTI:
       `OP_SLTIU:
       `OP_ANDI:
       `OP_ORI:
       `OP_XORI:
       `OP_LUI:
       `OP_LB:
       `OP_LH:
       `OP_LW:
       `OP_LBU:
       `OP_LHU:
       `OP_SB:
       `OP_SH:
       `OP_SW:*/
       default:
         begin
            ctrl_RI = 1'b1;
         end
     endcase // case(op)
   end

endmodule
