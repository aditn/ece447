/***************************************************************/
/*                                                             */
/*   MIPS-32 Instruction Level Simulator                       */
/*                                                             */
/*   ECE 447                                                   */
/*   Carnegie Mellon University                                */
/*                                                             */
/***************************************************************/

/* Lab 1.a Spring 2016
 * Complete the execute() function below by adding cases
 * for the missing opcodes and subocodes (see mips.h). 
 *
 * **NOTE*** You are only allowed to make modifications in the 
 * three specifically designated regions.
 */

#include <stdio.h>
#include "shell.h"
#include "mips.h"

uint32_t dcd_op;     /* decoded opcode */
uint32_t dcd_rs;     /* decoded rs operand */
uint32_t dcd_rt;     /* decoded rt operand */
uint32_t dcd_rd;     /* decoded rd operand */
uint32_t dcd_shamt;  /* decoded shift amount */
uint32_t dcd_funct;  /* decoded function */
uint32_t dcd_imm;    /* decoded immediate value */
uint32_t dcd_target; /* decoded target address */
int      dcd_se_imm; /* decoded sign-extended immediate value */
uint32_t inst;       /* machine instruction */


uint32_t sign_extend_b2w(uint8_t c)
{
  return (c & 0x80) ? (c | 0xffffff80) : c;
}

uint32_t sign_extend_h2w(uint16_t c)
{
  return (c & 0x8000) ? (c | 0xffff8000) : c;
}

/****************************************************************/
/*** you may add your own auxiliary functions below this line ***/

//blank




/*** you may add your own auxiliary functions above this line ***/
/****************************************************************/

void process_instruction()
{
  /*  function: process_instruction
   *  
   *    Process one instruction at a time  
   */

  /* fetch the 4 bytes of the current instruction */
  inst = mem_read_32(CURRENT_STATE.PC);

  /* decode an instruction into fields */
  dcd_op     = (inst >> 26) & 0x3F;
  dcd_rs     = (inst >> 21) & 0x1F;
  dcd_rt     = (inst >> 16) & 0x1F;
  dcd_rd     = (inst >> 11) & 0x1F;
  dcd_shamt  = (inst >> 6) & 0x1F;
  dcd_funct  = (inst >> 0) & 0x3F;
  dcd_imm    = (inst >> 0) & 0xFFFF;
  dcd_se_imm = sign_extend_h2w(dcd_imm);
  dcd_target = (inst >> 0) & ((1UL << 26) - 1);

  /* update architectural state according to instruction */
  switch (dcd_op) {
  case OP_SPECIAL:
    { 
      switch (dcd_funct) {
        case SUBOP_ADD: 
        case SUBOP_ADDU: 
          if (dcd_rd!=0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rs] + CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SYSCALL:
          if (CURRENT_STATE.REGS[2] == 10)
            RUN_BIT = 0;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
/*************************************************************/
/*** specify the remaining dcd_funct cases below this line ***/
        case SUBOP_SLL:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rt] << dcd_shamt;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SRL:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rt] >> dcd_shamt;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SRA:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = ((int32_t) CURRENT_STATE.REGS[dcd_rt]) >> dcd_shamt;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SLLV:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rt] << CURRENT_STATE.REGS[dcd_rs];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
       case SUBOP_SRLV:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rt] >> CURRENT_STATE.REGS[dcd_rs];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SRAV:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = ((int32_t) CURRENT_STATE.REGS[dcd_rt]) >> CURRENT_STATE.REGS[dcd_rs];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_JR:
          NEXT_STATE.PC = CURRENT_STATE.REGS[dcd_rs];
        case SUBOP_JALR:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.PC + 4; //is dcd_rd assumed to be 31?
          NEXT_STATE.PC = CURRENT_STATE.REGS[dcd_rs];
        case SUBOP_SUB:
           //same as SUBU
        case SUBOP_SUBU:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rs] - CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_AND:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rs] & CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_OR:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rs] | CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_XOR:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.REGS[dcd_rs] ^ CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_NOR:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = ~(CURRENT_STATE.REGS[dcd_rs] | CURRENT_STATE.REGS[dcd_rt]);
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SLT:
          if (dcd_rd != 0) {
            if ((int32_t) CURRENT_STATE.REGS[dcd_rs] < (int32_t) CURRENT_STATE.REGS[dcd_rt])
              NEXT_STATE.REGS[dcd_rd] = 1;
            else
              NEXT_STATE.REGS[dcd_rd] = 0;
          }
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_SLTU:
          if (dcd_rd != 0) {
            if (CURRENT_STATE.REGS[dcd_rs] < CURRENT_STATE.REGS[dcd_rt])
              NEXT_STATE.REGS[dcd_rd] = 1;
            else
              NEXT_STATE.REGS[dcd_rd] = 0;
          }
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MULT:
          NEXT_STATE.HI = (uint32_t) (((int64_t) NEXT_STATE.REGS[dcd_rs] * (int64_t) NEXT_STATE.REGS[dcd_rt]) >> 32);
          NEXT_STATE.LO = (uint32_t) (((int64_t) NEXT_STATE.REGS[dcd_rs] * (int64_t) NEXT_STATE.REGS[dcd_rt]));
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MFHI:
          if (dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.HI;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MFLO:
          if(dcd_rd != 0)
            NEXT_STATE.REGS[dcd_rd] = CURRENT_STATE.LO;
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MTHI:
          NEXT_STATE.HI = CURRENT_STATE.REGS[dcd_rs];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MTLO:
          NEXT_STATE.LO = CURRENT_STATE.REGS[dcd_rs];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_MULTU:
          NEXT_STATE.HI = (uint32_t) (((uint64_t) NEXT_STATE.REGS[dcd_rs] * (uint64_t) NEXT_STATE.REGS[dcd_rt]) >> 32);
          NEXT_STATE.LO = (uint32_t) (((uint64_t) NEXT_STATE.REGS[dcd_rs] * (uint64_t) NEXT_STATE.REGS[dcd_rt]));
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_DIV:
          NEXT_STATE.LO = (uint32_t) ((int32_t) CURRENT_STATE.REGS[dcd_rs] / (int32_t) CURRENT_STATE.REGS[dcd_rt]);
          NEXT_STATE.HI = (uint32_t) ((int32_t) CURRENT_STATE.REGS[dcd_rs] % (int32_t) CURRENT_STATE.REGS[dcd_rt]);
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;
        case SUBOP_DIVU:
          NEXT_STATE.LO = CURRENT_STATE.REGS[dcd_rs] / CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.HI = CURRENT_STATE.REGS[dcd_rs] % CURRENT_STATE.REGS[dcd_rt];
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
          break;

/*** specify the remaining dcd_funct cases above this line ***/
/*************************************************************/
        default:
          printf("Encountered unimplemented subopcode (0x%x). Ending simulation...\n\n", dcd_funct);
          RUN_BIT = 0;
      } /* switch (dcd_funct) */
    } /* case OP_SPECIAL */
    break;
  case OP_ADDI:
  case OP_ADDIU:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
/**********************************************************/
/*** specify the remaining dcd_op cases below this line ***/
  case OP_BRSPEC:
  {
     switch (dcd_funct) {
      case BROP_BLTZ:
        if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) < 0)
          NEXT_STATE.PC = CURRENT_STATE.PC + (dcd_se_imm<<2);
        else
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
        break;
      case BROP_BGEZ:
        if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) >= 0)
          NEXT_STATE.PC = CURRENT_STATE.PC + (dcd_se_imm<<2);
        else
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
        break;
      case BROP_BLTZAL:
          NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
        if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) < 0){
          NEXT_STATE.PC = CURRENT_STATE.PC + (dcd_se_imm<<2);
        }
        else
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
        break;
      case BROP_BGEZAL:
          NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
        if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) >= 0){
          NEXT_STATE.PC = CURRENT_STATE.PC + (dcd_se_imm<<2);
        }
        else
          NEXT_STATE.PC = CURRENT_STATE.PC + 4;
        break;
    }
  }
  }
  case OP_J:
    NEXT_STATE.PC = (CURRENT_STATE.PC & 0xF0000000) + (dcd_target<<2);
    break;
  case OP_JAL:
    NEXT_STATE.REGS[31] = CURRENT_STATE.PC + 4;
    NEXT_STATE.PC = (CURRENT_STATE.PC & 0xF0000000) + (dcd_target<<2);
    break;
  case OP_BEQ:
    if (CURRENT_STATE.REGS[dcd_rs] == CURRENT_STATE.REGS[dcd_rt])
      NEXT_STATE.PC = CURRENT_STATE.PC + 4 + (dcd_se_imm<<2);
    else
      NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_BNE:
    if (CURRENT_STATE.REGS[dcd_rs] != CURRENT_STATE.REGS[dcd_rt])
      NEXT_STATE.PC = CURRENT_STATE.PC + 4 + (dcd_se_imm<<2);
    else
      NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_BLEZ:
    if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) <= 0)
      NEXT_STATE.PC = CURRENT_STATE.PC + 4 + (dcd_se_imm<<2);
    else
      NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_BGTZ:
    if (((int32_t) CURRENT_STATE.REGS[dcd_rs]) > 0)
      NEXT_STATE.PC = CURRENT_STATE.PC + 4 + (dcd_se_imm<<2);
    else
      NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_SLTI:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = (((int32_t) CURRENT_STATE.REGS[dcd_rs]) < 0);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_SLTIU:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = (CURRENT_STATE.REGS[dcd_rs] < dcd_se_imm);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_ANDI:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = CURRENT_STATE.REGS[dcd_rs] + dcd_imm;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_ORI:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = CURRENT_STATE.REGS[dcd_rs] | dcd_imm;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_XORI:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = CURRENT_STATE.REGS[dcd_rs] ^ dcd_imm;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LUI:
    if (dcd_rt!=0)
      NEXT_STATE.REGS[dcd_rt] = (dcd_se_imm<<16) & 0xFFFF0000;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LB: //need 32bit addr
     if (dcd_rt != 0)
      NEXT_STATE.REGS[dcd_rt] = sign_extend_b2w((uint8_t) (mem_read_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm) & 0xFF));
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LH:
    if (dcd_rt != 0)
      NEXT_STATE.REGS[dcd_rt] = sign_extend_h2w((uint16_t) (mem_read_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm) & 0xFFFF));
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LW:
    if (dcd_rt != 0)
      NEXT_STATE.REGS[dcd_rt] = mem_read_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LBU:
    if (dcd_rt != 0)
      NEXT_STATE.REGS[dcd_rt] = mem_read_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm) & 0xFF;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_LHU:
    if (dcd_rt != 0)
      NEXT_STATE.REGS[dcd_rt] = mem_read_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm) & 0xFFFF;
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_SB:
    mem_write_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm, CURRENT_STATE.REGS[dcd_rt] & 0xFF);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_SH:
    mem_write_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm, CURRENT_STATE.REGS[dcd_rt] & 0xFFFF);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;
  case OP_SW:
    mem_write_32(CURRENT_STATE.REGS[dcd_rs] + dcd_se_imm, CURRENT_STATE.REGS[dcd_rt]);
    NEXT_STATE.PC = CURRENT_STATE.PC + 4;
    break;


/*** specify the remaining dcd_op cases above this line ***/
/*********************************************************/
  default:
    printf("Encountered unimplemented opcode (0x%x). Ending simulation...\n\n", dcd_op);
    RUN_BIT = 0;
  } /* switch (dcd_op) */
}
