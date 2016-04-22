// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

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

   wire [31:0] alu__ADD, alu__SUB, negVal;

   carry_select cs_3(alu__op1,alu__op2, , alu__ADD,);
   
   makeNegative mNeg(negVal, alu__op2);
   carry_select cs_4(alu__op1,negVal, , alu__SUB,);

   always_comb begin
    alu__out = 0;
    branchTrue = 0;
    case (alu__sel)
      `ALU_ADD: begin
        //alu__out = alu__op1+alu__op2;
        alu__out = alu__ADD;
      end
      `ALU_SUB:
        begin //check if branch condition is met
          alu__out = alu__SUB;
          //alu__out = alu__op1-alu__op2;
          /*case(brcond)
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
          endcase*/
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

module makeNegative (neg, valIn);
  input [31:0]  valIn;
  output [31:0] neg;

  wire [31:0] temp;
  
  assign temp = ~valIn;
  carry_select csNEG(32'hffffffff,temp, , neg,);

endmodule