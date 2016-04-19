////
//// forwardData: module for forwarding data to prevent RAW hazards
////
//// wr_reg_EX, wr_reg_MEM (inputs) - are reg numbers at different stages
//// dcd_rs, dcd_rt (inputs) - the registers the next instruction is reading
//// ctrl_we_EX, ctrl_we_MEM (inputs) - register write enable bits at different stages
//// rsfwd, rtfwd (output) - selects what data to forward to rs and rt data outputs
////

module forwardData(
  //input logic [31:0] pc_ID_1, pc_EX_1, pc_MEM_1, pc_ID_2, pc_EX_2, pc_MEM_2,
  input logic [4:0] wr_reg_EX_1, wr_reg_MEM_1, dcd_rt_1, dcd_rs_1,
  input logic [4:0] wr_reg_EX_2, wr_reg_MEM_2, dcd_rt_2, dcd_rs_2,
  input logic ctrl_we_EX_1, ctrl_we_MEM_1, ctrl_we_EX_2, ctrl_we_MEM_2,
  output logic [2:0] rsfwd_1, rtfwd_1, rsfwd_2, rtfwd_2);

  always_comb begin
    rsfwd_1 = 3'b0;
    rtfwd_1 = 3'b0;
    rsfwd_2 = 3'b0;
    rtfwd_2 = 3'b0;
    if (ctrl_we_MEM_1 != 0) begin
      if ((dcd_rs_1 != 0) && (dcd_rs_1 == wr_reg_MEM_1)) begin
        rsfwd_1 = 3'b100;
      end
      if ((dcd_rt_1 != 0) && (dcd_rt_1 == wr_reg_MEM_1)) begin
        rtfwd_1 = 3'b100;
      end
      if ((dcd_rs_2 != 0) && (dcd_rs_2 == wr_reg_MEM_1)) begin
        rsfwd_2 = 3'b100;
      end
      if ((dcd_rt_2 != 0) && (dcd_rt_2 == wr_reg_MEM_1)) begin
        rtfwd_2 = 3'b100;
      end
    end
    if (ctrl_we_MEM_2 != 0) begin
      if ((dcd_rs_1 != 0) && (dcd_rs_1 == wr_reg_MEM_2)) begin
        rsfwd_1 = 3'b011;
      end
      if ((dcd_rt_1 != 0) && (dcd_rt_1 == wr_reg_MEM_2)) begin
        rtfwd_1 = 3'b011;
      end
      if ((dcd_rs_2 != 0) && (dcd_rs_2 == wr_reg_MEM_2)) begin
        rsfwd_2 = 3'b011;
      end
      if ((dcd_rt_2 != 0) && (dcd_rt_2 == wr_reg_MEM_2)) begin
        rtfwd_2 = 3'b011;
      end
    end
    if (ctrl_we_EX_1 != 0) begin
      if ((dcd_rs_1 != 0) && (dcd_rs_1 == wr_reg_EX_1)) begin
        rsfwd_1 = 3'b010;
      end
      if ((dcd_rt_1 != 0) && (dcd_rt_1 == wr_reg_EX_1)) begin
        rtfwd_1 = 3'b010;
      end
      if ((dcd_rs_2 != 0) && (dcd_rs_2 == wr_reg_EX_1)) begin
        rsfwd_2 = 3'b010;
      end
      if ((dcd_rt_2 != 0) && (dcd_rt_2 == wr_reg_EX_1)) begin
        rtfwd_2 = 3'b010;
      end
    end
    if (ctrl_we_EX_2 != 0) begin
      if ((dcd_rs_1 != 0) && (dcd_rs_1 == wr_reg_EX_2)) begin
        rsfwd_1 = 3'b001;
      end
      if ((dcd_rt_1 != 0) && (dcd_rt_1 == wr_reg_EX_2)) begin
        rtfwd_1 = 3'b001;
      end
      if ((dcd_rs_2 != 0) && (dcd_rs_2 == wr_reg_EX_2)) begin
        rsfwd_2 = 3'b001;
      end
      if ((dcd_rt_2 != 0) && (dcd_rt_2 == wr_reg_EX_2)) begin
        rtfwd_2 = 3'b001;
      end
    end
  end

endmodule