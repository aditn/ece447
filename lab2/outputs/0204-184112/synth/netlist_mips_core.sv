
module mips_core_DW01_add_0 ( A, B, CI, SUM, CO );
  input [31:0] A;
  input [31:0] B;
  output [31:0] SUM;
  input CI;
  output CO;
  wire   \carry[31] , \carry[30] , \carry[29] , \carry[28] , \carry[27] ,
         \carry[26] , \carry[25] , \carry[24] , \carry[23] , \carry[22] ,
         \carry[21] , \carry[20] , \carry[19] , \carry[18] , \carry[17] ,
         \carry[16] , \carry[15] , \carry[14] , \carry[13] , \carry[12] ,
         \carry[11] , \carry[10] , \carry[9] , \carry[8] , \carry[7] ,
         \carry[6] , \carry[5] , \carry[4] ;
  tri   \A[31] ;
  tri   \A[30] ;
  tri   \A[29] ;
  tri   \A[28] ;
  tri   \A[27] ;
  tri   \A[26] ;
  tri   \A[25] ;
  tri   \A[24] ;
  tri   \A[23] ;
  tri   \A[22] ;
  tri   \A[21] ;
  tri   \A[20] ;
  tri   \A[19] ;
  tri   \A[18] ;
  tri   \A[17] ;
  tri   \A[16] ;
  tri   \A[15] ;
  tri   \A[14] ;
  tri   \A[13] ;
  tri   \A[12] ;
  tri   \A[11] ;
  tri   \A[10] ;
  tri   \A[9] ;
  tri   \A[8] ;
  tri   \A[7] ;
  tri   \A[6] ;
  tri   \A[5] ;
  tri   \A[4] ;
  tri   \A[3] ;
  tri   \A[1] ;
  tri   \A[0] ;
  tri   \carry[3] ;
  assign SUM[1] = \A[1] ;
  assign \A[1]  = A[1];
  assign SUM[0] = \A[0] ;
  assign \A[0]  = A[0];
  assign \carry[3]  = A[2];

  XOR2X1 U1 ( .A(A[31]), .B(\carry[31] ), .Y(SUM[31]) );
  AND2X1 U2 ( .A(\carry[30] ), .B(A[30]), .Y(\carry[31] ) );
  XOR2X1 U3 ( .A(A[30]), .B(\carry[30] ), .Y(SUM[30]) );
  AND2X1 U4 ( .A(\carry[29] ), .B(A[29]), .Y(\carry[30] ) );
  XOR2X1 U5 ( .A(A[29]), .B(\carry[29] ), .Y(SUM[29]) );
  AND2X1 U6 ( .A(\carry[28] ), .B(A[28]), .Y(\carry[29] ) );
  XOR2X1 U7 ( .A(A[28]), .B(\carry[28] ), .Y(SUM[28]) );
  AND2X1 U8 ( .A(\carry[27] ), .B(A[27]), .Y(\carry[28] ) );
  XOR2X1 U9 ( .A(A[27]), .B(\carry[27] ), .Y(SUM[27]) );
  AND2X1 U10 ( .A(\carry[26] ), .B(A[26]), .Y(\carry[27] ) );
  XOR2X1 U11 ( .A(A[26]), .B(\carry[26] ), .Y(SUM[26]) );
  AND2X1 U12 ( .A(\carry[25] ), .B(A[25]), .Y(\carry[26] ) );
  XOR2X1 U13 ( .A(A[25]), .B(\carry[25] ), .Y(SUM[25]) );
  AND2X1 U14 ( .A(\carry[24] ), .B(A[24]), .Y(\carry[25] ) );
  XOR2X1 U15 ( .A(A[24]), .B(\carry[24] ), .Y(SUM[24]) );
  AND2X1 U16 ( .A(\carry[23] ), .B(A[23]), .Y(\carry[24] ) );
  XOR2X1 U17 ( .A(A[23]), .B(\carry[23] ), .Y(SUM[23]) );
  AND2X1 U18 ( .A(\carry[22] ), .B(A[22]), .Y(\carry[23] ) );
  XOR2X1 U19 ( .A(A[22]), .B(\carry[22] ), .Y(SUM[22]) );
  AND2X1 U20 ( .A(\carry[21] ), .B(A[21]), .Y(\carry[22] ) );
  XOR2X1 U21 ( .A(A[21]), .B(\carry[21] ), .Y(SUM[21]) );
  AND2X1 U22 ( .A(\carry[20] ), .B(A[20]), .Y(\carry[21] ) );
  XOR2X1 U23 ( .A(A[20]), .B(\carry[20] ), .Y(SUM[20]) );
  AND2X1 U24 ( .A(\carry[19] ), .B(A[19]), .Y(\carry[20] ) );
  XOR2X1 U25 ( .A(A[19]), .B(\carry[19] ), .Y(SUM[19]) );
  AND2X1 U26 ( .A(\carry[18] ), .B(A[18]), .Y(\carry[19] ) );
  XOR2X1 U27 ( .A(A[18]), .B(\carry[18] ), .Y(SUM[18]) );
  AND2X1 U28 ( .A(\carry[17] ), .B(A[17]), .Y(\carry[18] ) );
  XOR2X1 U29 ( .A(A[17]), .B(\carry[17] ), .Y(SUM[17]) );
  AND2X1 U30 ( .A(\carry[16] ), .B(A[16]), .Y(\carry[17] ) );
  XOR2X1 U31 ( .A(A[16]), .B(\carry[16] ), .Y(SUM[16]) );
  AND2X1 U32 ( .A(\carry[15] ), .B(A[15]), .Y(\carry[16] ) );
  XOR2X1 U33 ( .A(A[15]), .B(\carry[15] ), .Y(SUM[15]) );
  AND2X1 U34 ( .A(\carry[14] ), .B(A[14]), .Y(\carry[15] ) );
  XOR2X1 U35 ( .A(A[14]), .B(\carry[14] ), .Y(SUM[14]) );
  AND2X1 U36 ( .A(\carry[13] ), .B(A[13]), .Y(\carry[14] ) );
  XOR2X1 U37 ( .A(A[13]), .B(\carry[13] ), .Y(SUM[13]) );
  AND2X1 U38 ( .A(\carry[12] ), .B(A[12]), .Y(\carry[13] ) );
  XOR2X1 U39 ( .A(A[12]), .B(\carry[12] ), .Y(SUM[12]) );
  AND2X1 U40 ( .A(\carry[11] ), .B(A[11]), .Y(\carry[12] ) );
  XOR2X1 U41 ( .A(A[11]), .B(\carry[11] ), .Y(SUM[11]) );
  AND2X1 U42 ( .A(\carry[10] ), .B(A[10]), .Y(\carry[11] ) );
  XOR2X1 U43 ( .A(A[10]), .B(\carry[10] ), .Y(SUM[10]) );
  AND2X1 U44 ( .A(\carry[9] ), .B(A[9]), .Y(\carry[10] ) );
  XOR2X1 U45 ( .A(A[9]), .B(\carry[9] ), .Y(SUM[9]) );
  AND2X1 U46 ( .A(\carry[8] ), .B(A[8]), .Y(\carry[9] ) );
  XOR2X1 U47 ( .A(A[8]), .B(\carry[8] ), .Y(SUM[8]) );
  AND2X1 U48 ( .A(\carry[7] ), .B(A[7]), .Y(\carry[8] ) );
  XOR2X1 U49 ( .A(A[7]), .B(\carry[7] ), .Y(SUM[7]) );
  AND2X1 U50 ( .A(\carry[6] ), .B(A[6]), .Y(\carry[7] ) );
  XOR2X1 U51 ( .A(A[6]), .B(\carry[6] ), .Y(SUM[6]) );
  AND2X1 U52 ( .A(\carry[5] ), .B(A[5]), .Y(\carry[6] ) );
  XOR2X1 U53 ( .A(A[5]), .B(\carry[5] ), .Y(SUM[5]) );
  AND2X1 U54 ( .A(\carry[4] ), .B(A[4]), .Y(\carry[5] ) );
  XOR2X1 U55 ( .A(A[4]), .B(\carry[4] ), .Y(SUM[4]) );
  AND2X1 U56 ( .A(\carry[3] ), .B(A[3]), .Y(\carry[4] ) );
  XOR2X1 U57 ( .A(A[3]), .B(\carry[3] ), .Y(SUM[3]) );
  INVX1 U58 ( .A(\carry[3] ), .Y(SUM[2]) );
endmodule


module mips_core ( inst_addr, mem_addr, mem_data_in, mem_write_en, halted, clk, 
        inst_excpt, mem_excpt, inst, mem_data_out, rst_b );
  output [29:0] inst_addr;
  output [29:0] mem_addr;
  output [31:0] mem_data_in;
  output [3:0] mem_write_en;
  input [31:0] inst;
  input [31:0] mem_data_out;
  input clk, inst_excpt, mem_excpt, rst_b;
  output halted;
  wire   alusrc1, alusrc2, se, hi_en, lo_en, branchTrue, \_11_net_[31] ,
         \_11_net_[30] , \_11_net_[29] , \_11_net_[28] , \_11_net_[27] ,
         \_11_net_[26] , \_11_net_[25] , \_11_net_[24] , \_11_net_[23] ,
         \_11_net_[22] , \_11_net_[21] , \_11_net_[20] , \_11_net_[19] ,
         \_11_net_[18] , \_11_net_[17] , \_11_net_[16] , \_11_net_[15] ,
         \_11_net_[14] , \_11_net_[13] , \_11_net_[12] , \_11_net_[11] ,
         \_11_net_[10] , \_11_net_[9] , \_11_net_[8] , \_11_net_[7] ,
         \_11_net_[6] , \_11_net_[5] , \_11_net_[4] , \_11_net_[3] ,
         \_11_net_[2] , n6, n7;
  wire   [31:0] newpc;
  wire   [3:0] alu__sel;
  wire   [1:0] pcMuxSel;
  wire   [1:0] memtoreg;
  wire   [2:0] load_sel;
  wire   [2:0] brcond;
  wire   [1:0] store_sel;
  wire   [1:0] alu__out;
  wire   [31:0] alu_in1;
  wire   [31:0] alu_in2;
  wire   [31:0] hi_out;
  wire   [31:0] lo_out;
  wire   [31:0] imm;
  wire   [31:0] wr_dataMem;
  wire   [31:0] load_data;
  wire   [31:0] br_target;
  wire   [31:0] j_target;
  wire   [1:0] pcMuxSelFinal;
  tri   [29:0] inst_addr;
  tri   halted;
  tri   clk;
  tri   inst_excpt;
  tri   rst_b;
  tri   [1:0] pc;
  tri   ctrl_we;
  tri   ctrl_Sys;
  tri   ctrl_RI;
  tri   regdst;
  tri   jLink_en;
  tri   exception_halt;
  tri   load_ex_regs;
  tri   load_bva;
  tri   [4:0] cause_code;
  tri   \_5_net_[0] ;
  tri   syscall_halt;
  tri   [31:0] rs_data;
  tri   [31:0] rt_data;
  tri   [31:0] wr_data;
  tri   \_11_net_[1] ;
  tri   \_11_net_[0] ;
  tri   \wr_reg[4] ;
  tri   \wr_reg[3] ;
  tri   \wr_reg[2] ;
  tri   \wr_reg[1] ;
  tri   \wr_reg[0] ;
  tri   \wr_regNum[9] ;
  tri   \wr_regNum[8] ;
  tri   \wr_regNum[7] ;
  tri   \wr_regNum[6] ;
  tri   \wr_regNum[5] ;
  tri   \wr_regNum[4] ;
  tri   \wr_regNum[3] ;
  tri   \wr_regNum[31] ;
  tri   \wr_regNum[30] ;
  tri   \wr_regNum[2] ;
  tri   \wr_regNum[29] ;
  tri   \wr_regNum[28] ;
  tri   \wr_regNum[27] ;
  tri   \wr_regNum[26] ;
  tri   \wr_regNum[25] ;
  tri   \wr_regNum[24] ;
  tri   \wr_regNum[23] ;
  tri   \wr_regNum[22] ;
  tri   \wr_regNum[21] ;
  tri   \wr_regNum[20] ;
  tri   \wr_regNum[1] ;
  tri   \wr_regNum[19] ;
  tri   \wr_regNum[18] ;
  tri   \wr_regNum[17] ;
  tri   \wr_regNum[16] ;
  tri   \wr_regNum[15] ;
  tri   \wr_regNum[14] ;
  tri   \wr_regNum[13] ;
  tri   \wr_regNum[12] ;
  tri   \wr_regNum[11] ;
  tri   \wr_regNum[10] ;
  tri   \wr_regNum[0] ;
  tri   n8;
  tri   n9;
  wire   SYNOPSYS_UNCONNECTED__0, SYNOPSYS_UNCONNECTED__1;

  register_32_00400000 PCReg ( .q({inst_addr, pc}), .d(newpc), .clk(clk), 
        .enable(n7), .rst_b(rst_b) );
  register_32_00400004 PCReg2 ( .d(newpc), .clk(clk), .enable(n7), .rst_b(
        rst_b) );
  add_const_add_value4 NextPCAdder ( .in({inst_addr, pc}) );
  mips_decode Decoder ( .ctrl_we(ctrl_we), .ctrl_Sys(ctrl_Sys), .ctrl_RI(
        ctrl_RI), .alu__sel(alu__sel), .regdst(regdst), .pcMuxSel(pcMuxSel), 
        .jLink_en(jLink_en), .memtoreg(memtoreg), .alusrc1(alusrc1), .alusrc2(
        alusrc2), .se(se), .hi_en(hi_en), .lo_en(lo_en), .load_sel(load_sel), 
        .brcond(brcond), .store_sel(store_sel), .dcd_op(inst[31:26]), 
        .dcd_funct2(inst[5:0]), .dcd_rt(inst[20:16]) );
  mips_ALU ALU ( .alu__out({mem_addr, alu__out}), .branchTrue(branchTrue), 
        .alu__op1(alu_in1), .alu__op2(alu_in2), .alu__sel(alu__sel), .brcond(
        brcond) );
  exception_unit EU ( .exception_halt(exception_halt), .pc({inst_addr, pc}), 
        .rst_b(rst_b), .clk(clk), .load_ex_regs(load_ex_regs), .load_bva(
        load_bva), .cause(cause_code), .IBE(inst_excpt), .DBE(1'b0), .RI(
        ctrl_RI), .Ov(1'b0), .BP(1'b0), .AdEL_inst(\_5_net_[0] ), .AdEL_data(
        1'b0), .AdES(1'b0), .CpU(1'b0) );
  syscall_unit SU ( .syscall_halt(syscall_halt), .pc({inst_addr, pc}), .clk(
        clk), .Sys(ctrl_Sys), .r_v0({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b1, 
        1'b0}), .rst_b(rst_b) );
  register_width1_reset_value0 Halt ( .q(halted), .d(n6), .clk(clk), .enable(
        1'b1), .rst_b(rst_b) );
  register_width32_reset_value0_4 EPCReg ( .d({inst_addr, pc}), .clk(clk), 
        .enable(load_ex_regs), .rst_b(rst_b) );
  register_width32_reset_value0_3 CauseReg ( .d({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, cause_code, 1'b0, 1'b0}), .clk(clk), .enable(load_ex_regs), .rst_b(rst_b) );
  register_width32_reset_value0_2 BadVAddrReg ( .d({inst_addr, pc}), .clk(clk), 
        .enable(load_bva), .rst_b(rst_b) );
  regfile RegFile ( .p1(rs_data), .p2(rt_data), .p3(inst[25:21]), .p4(
        inst[20:16]), .p5({\wr_reg[4] , \wr_reg[3] , \wr_reg[2] , \wr_reg[1] , 
        \wr_reg[0] }), .p6(wr_data), .p7(ctrl_we), .p8(clk), .p9(rst_b), .p10(
        halted) );
  register_width32_reset_value0_1 hiReg ( .q(hi_out), .d(rs_data), .clk(clk), 
        .enable(hi_en), .rst_b(rst_b) );
  register_width32_reset_value0_0 loReg ( .q(lo_out), .d(rs_data), .clk(clk), 
        .enable(lo_en), .rst_b(rst_b) );
  mux2to1 regDest ( .p1({\wr_regNum[31] , \wr_regNum[30] , \wr_regNum[29] , 
        \wr_regNum[28] , \wr_regNum[27] , \wr_regNum[26] , \wr_regNum[25] , 
        \wr_regNum[24] , \wr_regNum[23] , \wr_regNum[22] , \wr_regNum[21] , 
        \wr_regNum[20] , \wr_regNum[19] , \wr_regNum[18] , \wr_regNum[17] , 
        \wr_regNum[16] , \wr_regNum[15] , \wr_regNum[14] , \wr_regNum[13] , 
        \wr_regNum[12] , \wr_regNum[11] , \wr_regNum[10] , \wr_regNum[9] , 
        \wr_regNum[8] , \wr_regNum[7] , \wr_regNum[6] , \wr_regNum[5] , 
        \wr_regNum[4] , \wr_regNum[3] , \wr_regNum[2] , \wr_regNum[1] , 
        \wr_regNum[0] }), .p2(inst[20:16]), .p3(inst[15:11]), .p4(regdst) );
  mux2to1_3 aluSrc1 ( .out(alu_in1), .in0(rs_data), .in1(rt_data), .sel(
        alusrc1) );
  mux2to1_2 aluSrc2 ( .out(alu_in2), .in0(rt_data), .in1(imm), .sel(alusrc2)
         );
  mux2to1_1 signext ( .out(imm), .in0({1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, inst[15:0]}), .in1({inst[15], inst[15], inst[15], inst[15], inst[15], inst[15], inst[15], 
        inst[15], inst[15], inst[15], inst[15], inst[15], inst[15], inst[15], 
        inst[15], inst[15], inst[15:0]}), .sel(se) );
  mux4to1_1 memToReg ( .out(wr_dataMem), .in0({mem_addr, alu__out}), .in1(
        load_data), .in2(hi_out), .in3(lo_out), .sel(memtoreg) );
  loader loader ( .load_data(load_data), .dcd_imm(inst[15:0]), .mem_data(
        mem_data_out), .load_sel(load_sel), .offset({mem_addr, alu__out}) );
  storer storer ( .store_data(mem_data_in), .mem_write_en(mem_write_en), 
        .rt_data(rt_data), .store_sel(store_sel), .offset({mem_addr, alu__out}) );
  mux4to1_0 pcMux ( .out(newpc), .in0({\_11_net_[31] , \_11_net_[30] , 
        \_11_net_[29] , \_11_net_[28] , \_11_net_[27] , \_11_net_[26] , 
        \_11_net_[25] , \_11_net_[24] , \_11_net_[23] , \_11_net_[22] , 
        \_11_net_[21] , \_11_net_[20] , \_11_net_[19] , \_11_net_[18] , 
        \_11_net_[17] , \_11_net_[16] , \_11_net_[15] , \_11_net_[14] , 
        \_11_net_[13] , \_11_net_[12] , \_11_net_[11] , \_11_net_[10] , 
        \_11_net_[9] , \_11_net_[8] , \_11_net_[7] , \_11_net_[6] , 
        \_11_net_[5] , \_11_net_[4] , \_11_net_[3] , \_11_net_[2] , 
        \_11_net_[1] , \_11_net_[0] }), .in1(br_target), .in2(rs_data), .in3({
        j_target[31:2], 1'b0, 1'b0}), .sel(pcMuxSelFinal) );
  adder brtarget ( .out(br_target), .in1({\_11_net_[31] , \_11_net_[30] , 
        \_11_net_[29] , \_11_net_[28] , \_11_net_[27] , \_11_net_[26] , 
        \_11_net_[25] , \_11_net_[24] , \_11_net_[23] , \_11_net_[22] , 
        \_11_net_[21] , \_11_net_[20] , \_11_net_[19] , \_11_net_[18] , 
        \_11_net_[17] , \_11_net_[16] , \_11_net_[15] , \_11_net_[14] , 
        \_11_net_[13] , \_11_net_[12] , \_11_net_[11] , \_11_net_[10] , 
        \_11_net_[9] , \_11_net_[8] , \_11_net_[7] , \_11_net_[6] , 
        \_11_net_[5] , \_11_net_[4] , \_11_net_[3] , \_11_net_[2] , 
        \_11_net_[1] , \_11_net_[0] }), .in2({imm[29:0], 1'b0, 1'b0}), .sub(
        1'b0) );
  concat conc ( .j_target({j_target[31:2], SYNOPSYS_UNCONNECTED__0, 
        SYNOPSYS_UNCONNECTED__1}), .cur_pc({inst_addr, pc}), .dcd_target(
        inst[25:0]) );
  muxSpecial choosePcMuxSel ( .pcMuxSelFinal(pcMuxSelFinal), .pcMuxSel(
        pcMuxSel), .branchTrue(branchTrue) );
  mux2to1_0 dataToReg ( .out(wr_data), .in0(wr_dataMem), .in1({\_11_net_[31] , 
        \_11_net_[30] , \_11_net_[29] , \_11_net_[28] , \_11_net_[27] , 
        \_11_net_[26] , \_11_net_[25] , \_11_net_[24] , \_11_net_[23] , 
        \_11_net_[22] , \_11_net_[21] , \_11_net_[20] , \_11_net_[19] , 
        \_11_net_[18] , \_11_net_[17] , \_11_net_[16] , \_11_net_[15] , 
        \_11_net_[14] , \_11_net_[13] , \_11_net_[12] , \_11_net_[11] , 
        \_11_net_[10] , \_11_net_[9] , \_11_net_[8] , \_11_net_[7] , 
        \_11_net_[6] , \_11_net_[5] , \_11_net_[4] , \_11_net_[3] , 
        \_11_net_[2] , \_11_net_[1] , \_11_net_[0] }), .sel(jLink_en) );
  mux2to1 regNumber ( .p1({\wr_reg[4] , \wr_reg[3] , \wr_reg[2] , \wr_reg[1] , 
        \wr_reg[0] }), .p2({\wr_regNum[31] , \wr_regNum[30] , \wr_regNum[29] , 
        \wr_regNum[28] , \wr_regNum[27] , \wr_regNum[26] , \wr_regNum[25] , 
        \wr_regNum[24] , \wr_regNum[23] , \wr_regNum[22] , \wr_regNum[21] , 
        \wr_regNum[20] , \wr_regNum[19] , \wr_regNum[18] , \wr_regNum[17] , 
        \wr_regNum[16] , \wr_regNum[15] , \wr_regNum[14] , \wr_regNum[13] , 
        \wr_regNum[12] , \wr_regNum[11] , \wr_regNum[10] , \wr_regNum[9] , 
        \wr_regNum[8] , \wr_regNum[7] , \wr_regNum[6] , \wr_regNum[5] , 
        \wr_regNum[4] , \wr_regNum[3] , \wr_regNum[2] , \wr_regNum[1] , 
        \wr_regNum[0] }), .p3({1'b1, 1'b1, 1'b1, 1'b1, 1'b1}), .p4(jLink_en)
         );
  mips_core_DW01_add_0 r50 ( .A({inst_addr, pc}), .B({1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 
        1'b0, 1'b1, 1'b0, 1'b0}), .CI(1'b0), .SUM({\_11_net_[31] , 
        \_11_net_[30] , \_11_net_[29] , \_11_net_[28] , \_11_net_[27] , 
        \_11_net_[26] , \_11_net_[25] , \_11_net_[24] , \_11_net_[23] , 
        \_11_net_[22] , \_11_net_[21] , \_11_net_[20] , \_11_net_[19] , 
        \_11_net_[18] , \_11_net_[17] , \_11_net_[16] , \_11_net_[15] , 
        \_11_net_[14] , \_11_net_[13] , \_11_net_[12] , \_11_net_[11] , 
        \_11_net_[10] , \_11_net_[9] , \_11_net_[8] , \_11_net_[7] , 
        \_11_net_[6] , \_11_net_[5] , \_11_net_[4] , \_11_net_[3] , 
        \_11_net_[2] , \_11_net_[1] , \_11_net_[0] }) );
  OR2X2 C18 ( .A(pc[1]), .B(pc[0]), .Y(\_5_net_[0] ) );
  INVX1 U9 ( .A(n6), .Y(n7) );
  OR2X2 U10 ( .A(exception_halt), .B(syscall_halt), .Y(n6) );
endmodule

