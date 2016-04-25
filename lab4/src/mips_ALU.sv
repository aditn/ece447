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

   wire [31:0] alu__ADD, alu__SUB, negVal, alu__SRA, alu__SRAV;

   carry_select cs_3(alu__op1,alu__op2,1'b0 , alu__ADD,);
   
   makeNegative mNeg(negVal, alu__op2);
   carry_select cs_4(alu__op1,negVal, 1'b0, alu__SUB,);

   shiftRightAr sra($signed(alu__op1), alu__op2[10:6], alu__SRA);
   shiftRightAr srav($signed(alu__op2), alu__op1[4:0], alu__SRAV);

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
          //alu__out = alu__SUB;
          //alu__out = alu__op1-alu__op2;
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
              begin
              alu__out = alu__SUB;
              //alu__out = alu__op1-alu__op2;
              end
          endcase
        end
      `ALU_SLL:
        //shift by value in bits [10:6] of immediate
        alu__out = alu__op1<<{27'b0, alu__op2[10:6]};
      `ALU_SRL:
        alu__out = alu__op1>>{27'b0, alu__op2[10:6]};
      `ALU_SRA:
        //signed arithmetic shift
        //alu__out = $signed($signed(alu__op1) >>> {27'b0, alu__op2[10:6]});
        alu__out = $signed(alu__SRA);
      `ALU_SLLV:
        alu__out = alu__op2<<{27'b0, alu__op1[4:0]};
      `ALU_SRLV:
        alu__out = alu__op2>>{27'b0, alu__op1[4:0]};
      `ALU_SRAV:
        //alu__out = $signed($signed(alu__op2) >>> {27'b0, alu__op1[4:0]});
        alu__out = $signed(alu__SRAV);
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

module shiftRightAr(a, shiftCnt, z);
  input [31:0] a;
  input [4:0] shiftCnt;
  output [31:0] z;
  wire [31:0] d0, d1, d2, d3, notA;
  
  assign notA = ~a;
  mux2to1L32 m21_0 (notA,{notA[31], notA[31:1]}, shiftCnt[0], d0);
  mux2to1L32 m21_1 (d0,{{ 2{a[31]}}, d0[31:2]}, shiftCnt[1], d1);
  mux2to1L32 m21_2 (d1,{{ 4{notA[31]}}, d1[31:4]}, shiftCnt[2],d2);
  mux2to1L32 m21_3 (d2,{{ 8{a[31]}}, d2[31:8]}, shiftCnt[3], d3);
  mux2to1L32 m21_4 (d3,{{16{notA[31]}}, d3[31:16]},shiftCnt[4],z);
endmodule

module mux2to1L32 (a, b, s, z);
  input [31:0] a, b;
  input s;
  output [31:0] z;
  assign z = ~(s ? b : a);
endmodule


module makeNegative (neg, valIn);
  input [31:0]  valIn;
  output [31:0] neg;

  wire [31:0] temp;
  /*always_comb begin
    $display("valIn:%x,%d,%b",valIn,valIn,valIn);
    $display("temp:%x,%d,%b",temp,temp,temp);
    $display("neg:%x,%d,%b",neg,neg,neg);
  end*/
  assign temp = ~valIn;
  carry_select csNEG(32'd1,temp,1'b0, neg,);

endmodule
