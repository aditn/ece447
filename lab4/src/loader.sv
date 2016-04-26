
// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

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
      input logic [15:0] dcd_imm_1,dcd_imm_2,
      input logic [31:0] mem_data,
      input logic [2:0] load_sel_1,load_sel_2,
      input logic [31:0] offset_1,offset_2);

    //shift the data by the offset number of bytes
    logic [31:0] data;
    logic [2:0]  load_sel;
    logic [31:0] offset;
    logic [15:0] dcd_imm;
    always_comb begin
      if (load_sel_1 != `NO_LOAD) begin
        load_sel = load_sel_1;
        offset = offset_1;
        dcd_imm = dcd_imm_1;
      end
      else if (load_sel_2 != `NO_LOAD) begin
        load_sel = load_sel_2;
        offset = offset_2;
        dcd_imm = dcd_imm_2;
      end
      else begin
        load_sel = `NO_LOAD;
        offset = 31'd0;
      end  
    end

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
        `NO_LOAD:
          load_data = 32'hxxxx;
      endcase
    end
endmodule
