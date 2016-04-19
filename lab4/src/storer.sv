////
//// storer: operates on data for store instructions
////
//// store_data (output) - data to load into registers
//// mem_en (output) - shifted memory write enable bits
//// rt_data   (input)  - data from rt register
//// store_sel  (input)  - selects which store operation to perform
//// offset    (input)  - byte offset
//// mem_write_en (input) - memory enable bits from control module
////
module storer (
      output logic [31:0] store_data,
      output logic [3:0] mem_write_en,
      input logic [31:0] rt_data_1, rt_data_2,
      input logic [1:0] store_sel_1, store_sel_2,
      input logic [31:0] offset_1, offset_2,
      input logic [3:0] mem_en_1, mem_en_2);

    logic [3:0] mem_en;
    logic [31:0] rt_data;
    logic [1:0] store_sel;
    logic [31:0] offset;
    always_comb begin
      if (mem_en_1 != 4'd0) begin
        mem_en = mem_en_1;
        rt_data = rt_data_1;
        store_sel = store_sel_1;
        offset = offset_1;
      end
      else if (mem_en_2 != 4'd0) begin
        mem_en = mem_en_2;
        rt_data = rt_data_2;
        store_sel = store_sel_2;
        offset = offset_2;
      end
      else begin
        mem_en = 3'd0;
        rt_data = 32'bx;
        store_sel = 2'bx;
        offset = 32'bx;
      end
    end

    always_comb begin
      case(store_sel)
        `ST_SB: //place data in top bits of word and shift right by offset number of bytes
          begin
            store_data = {24'b0, rt_data[7:0]} << ((offset & 32'h3) * 8);
            mem_write_en = mem_en >> (offset & 32'h3);
          end
        `ST_SH:
          begin
            store_data = {16'b0, rt_data[15:0]} << ((offset & 32'h3) * 8);
            mem_write_en = mem_en >> (offset & 32'h3);
          end
        `ST_SW:
          begin
            store_data = rt_data;
            mem_write_en = mem_en;
          end
        default:
          begin
            store_data = 32'hxxxx;
            mem_write_en = 4'b0000;
          end
      endcase
    end

endmodule