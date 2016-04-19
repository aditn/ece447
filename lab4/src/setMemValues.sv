
// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

module setMemValues(
   output logic [29:0] mem_addr,
   output logic [31:0] mem_data_in,
   input logic [31:0] alu__out_MEM_1, alu__out_MEM_2, store_data,
   input logic [3:0] mem_write_en_MEM_1,mem_write_en_MEM_2,
   input logic [2:0] load_sel_MEM_1, load_sel_MEM_2);
   
   /*output [29:0] mem_addr;
   output [31:0] mem_data_in;
   input  [31:0] alu__out_MEM_1,alu__out_MEM_2, store_data;
   input  [3:0]  mem_write_en_MEM_1, mem_write_en_MEM_2;*/

   always_comb begin
     mem_addr = 30'bx;
     mem_data_in = store_data;
     if ((mem_write_en_MEM_1 != 4'd0) | (load_sel_MEM_1 != `NO_LOAD)) begin
       //$display("mem_write_en_MEM_1 != 0");
       mem_addr = alu__out_MEM_1[31:2];
     end
     else if ((mem_write_en_MEM_2 != 4'd0) | (load_sel_MEM_2 != `NO_LOAD)) begin
      //$display("mem_write_en_MEM_2 != 0");
       mem_addr = alu__out_MEM_2[31:2];
     end
     
   end
endmodule