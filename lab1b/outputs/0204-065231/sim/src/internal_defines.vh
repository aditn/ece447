////
//// Internal signal constants
////

// ALU
`define ALU_ADD      4'b0000
`define ALU_SUB      4'b0001
`define ALU_SLL      4'b0010
`define ALU_SRL      4'b0011
`define ALU_SRA      4'b0100
`define ALU_SLLV     4'b0101
`define ALU_SRLV     4'b0110
`define ALU_SRAV     4'b0111
`define ALU_AND      4'b1000
`define ALU_OR       4'b1001
`define ALU_XOR      4'b1010
`define ALU_NOR      4'b1011
`define ALU_SLT      4'b1100



// Load instruction for loader module
`define LOAD_LUI     3'b000
`define LOAD_LB      3'b001
`define LOAD_LH      3'b010
`define LOAD_LW      3'b011
`define LOAD_LBU     3'b100
`define LOAD_LHU     3'b101

`define ST_SB 2'b00
`define ST_SH 2'b01
`define ST_SW 2'b10

