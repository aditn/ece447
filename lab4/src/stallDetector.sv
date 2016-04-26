////
//// stallDetector: module for enabling stalls if RAW, WAR, WAW hazard
////
//// pc_ID, pc_EX (inputs) - pc values at different stages
//// wr_reg_EX, wr_reg_MEM (inputs) - are reg numbers at different stages
//// dcd_rs, dcd_rt (inputs) - the registers the next instruction is reading
//// regdst (input) - destination register
//// mem_en (input) - the memory write enable bits at ID stage
//// ctrl_we, ctrl_we_EX (inputs) - register write enable bits at different stages
//// stall (input) - signal that indicates an instruction is currently being stalled
//// IFen, IDen, EXen (outputs) - register enable bits at the IF, ID, and EX stages
//// CDen (output) - enable bit for countdown register
//// CDAmt (output) - number of clock cycles to stall
////
module stallDetector(
  input logic [31:0] pc_ID_1, pc_EX_1, rt_data_MEM_1,
  input logic [4:0] wr_reg_1, wr_reg_EX_1, wr_reg_MEM_1, dcd_rt_1, dcd_rs_1,
  input logic [3:0] mem_en_1,
  input logic ctrl_we_1, ctrl_we_EX_1, regdst_1, ctrl_Sys_MEM_1,
  input logic [31:0] pc_ID_2, pc_EX_2, rt_data_MEM_2,
  input logic [4:0] wr_reg_2, wr_reg_EX_2, wr_reg_MEM_2, dcd_rt_2, dcd_rs_2,
  input logic [3:0] mem_en_2,
  input logic ctrl_we_2, ctrl_we_EX_2, regdst_2, ctrl_Sys_MEM_2, EXenFlush_2,
  input logic stall_1, stall_2, load_stall_1, load_stall_EX_1, load_stall_2, load_stall_EX_2,
  input logic [1:0] store_sel_1, store_sel_2, pcMuxSel_2,
  output logic IFen_1, IDen_1, EXen_1, MEMen_1, IFen_2, IDen_2, EXen_2, MEMen_2, WBen_2, IDclr, IDswap, CDen,
  output logic [2:0] CDAmt);
  
  always_comb begin
    IFen_1 = 1'b1;
    IDen_1 = 1'b1;
    IDclr = 1'b1;
    IDswap = 1'b0;
    EXen_1 = 1'b1;
    MEMen_1 = 1'b1;
    IFen_2 = 1'b1;
    IDen_2 = 1'b1;
    EXen_2 = 1'b1;
    MEMen_2 = 1'b1;
    WBen_2 = 1'b1;
    CDen = 1'b0;
    CDAmt = 3'b0;

    //$display("stall_1: %b, stall_2: %b, load_stall_1: %x, load_stall_2: %x, lEX1: %x, lEX2: %x", stall_1, stall_2, load_stall_1, load_stall_2, load_stall_EX_1, load_stall_EX_2);
    //$display("IDen1: %b, IDclr: %b", IDen_1, IDclr);

    if(stall_1==1'b1) begin
      IFen_1 = 1'b0;
      IDen_1 = 1'b0;
      EXen_1 = 1'b0;
      IFen_2 = 1'b0;
      IDen_2 = 1'b0;
      EXen_2 = 1'b0;
    end

    else if(stall_2==1'b1) begin
      IFen_2 = 1'b0;
      IDen_2 = 1'b0;
      EXen_2 = 1'b0;
      IFen_1 = 1'b0;
      IDen_1 = 1'b0;
      IDclr = 1'b0;
    end
    else begin
      if(stall_1==1'b0 && stall_2==1'b0) begin
        if(ctrl_Sys_MEM_1==1'b1 && rt_data_MEM_1==32'ha) begin
          MEMen_1 = 1'b0;
          MEMen_2 = 1'b0;
          WBen_2 = 1'b0;
        end
        else if(ctrl_Sys_MEM_2==1'b1 && rt_data_MEM_2==32'ha) begin
          MEMen_1 = 1'b0;
          MEMen_2 = 1'b0;
        end
      end
      if(stall_1==1'b0) begin
        if(load_stall_EX_1==1'b1) begin
          if((ctrl_we_EX_1!=0) && (((regdst_1==1) && (dcd_rt_1!=0) && (dcd_rt_1==wr_reg_EX_1)) || ((dcd_rs_1!=0) && (dcd_rs_1==wr_reg_EX_1)))) begin
            IFen_1 = 1'b0;
            IDen_1 = 1'b0;
            EXen_1 = 1'b0;
            IFen_2 = 1'b0;
            IDen_2 = 1'b0;
            EXen_2 = 1'b0;
          end

        end
        if(load_stall_EX_2==1'b1 && (pc_ID_1>pc_EX_2)) begin
          if((ctrl_we_EX_2!=0) && (((regdst_1==1) && (dcd_rt_1!=0) && (dcd_rt_1==wr_reg_EX_2)) || ((dcd_rs_1!=0) && (dcd_rs_1==wr_reg_EX_2)))) begin
            IFen_1 = 1'b0;
            IDen_1 = 1'b0;
            EXen_1 = 1'b0;
            IFen_2 = 1'b0;
            IDen_2 = 1'b0;
            EXen_2 = 1'b0;
          end
        end
        if((load_stall_1==1'b1 || mem_en_1!=4'b0) && (load_stall_2==1'b1 || mem_en_2!=4'b0) && (pc_ID_1>pc_ID_2)) begin
          $display("this");
          IFen_1 = 1'b0;
          IDen_1 = 1'b0;
          EXen_1 = 1'b0;
          IFen_2 = 1'b0;
          IDen_2 = 1'b0;
        end
      end
   //$display ("we_1: %x, we_2: %x, dcd_rt_2: %x, dcd_rs_2: %x, wr_reg_1: %x", ctrl_we_1, ctrl_we_2, dcd_rt_2, dcd_rs_2, wr_reg_1);
   //$display ("pc_ID_2: %x, pc_ID_1: %x, regdst_2: %x", pc_ID_2, pc_ID_1, regdst_2);
      if(stall_2==1'b0 && IDen_1==1'b1) begin
        if(ctrl_we_1!=0 /*&& ctrl_we_2!=0*/ && pc_ID_2>pc_ID_1 && EXenFlush_2==1'b1 && (((regdst_2==1) && (dcd_rt_2!=0) && (dcd_rt_2==wr_reg_1)) || ((dcd_rs_2!=0) && (dcd_rs_2==wr_reg_1)))) begin
          /*IFen_2 = 1'b0;
          IDen_2 = 1'b0;
          EXen_2 = 1'b0;
          IFen_1 = 1'b0;
          IDen_1 = 1'b0;
          IDclr = 1'b0;*/
          IDswap = 1'b1;
          EXen_2 = 1'b0;
          //swap condition
        end
        if(pcMuxSel_2!=2'b00 && EXenFlush_2==1'b1) begin
          IDswap = 1'b1;
          EXen_2 = 1'b0;
        end
        if(load_stall_EX_2==1'b1) begin
          if((ctrl_we_EX_2!=0) && (((regdst_2==1) && (dcd_rt_2!=0) && (dcd_rt_2==wr_reg_EX_2)) || ((dcd_rs_2!=0) && (dcd_rs_2==wr_reg_EX_2)))) begin
            IFen_2 = 1'b0;
            IDen_2 = 1'b0;
            EXen_2 = 1'b0;
            IFen_1 = 1'b0;
            IDen_1 = 1'b0;
            IDclr = 1'b0;
          end
        end
        if(load_stall_EX_1==1'b1 && (pc_ID_2>pc_EX_1)) begin
          if((ctrl_we_EX_1!=0) && (((regdst_2==1) && (dcd_rt_2!=0) && (dcd_rt_2==wr_reg_EX_1)) || ((dcd_rs_2!=0) && (dcd_rs_2==wr_reg_EX_1)))) begin
            IFen_2 = 1'b0;
            IDen_2 = 1'b0;
            EXen_2 = 1'b0;
            IFen_1 = 1'b0;
            IDen_1 = 1'b0;
            IDclr = 1'b0;
          end
        end
        if((load_stall_1==1'b1 || mem_en_1!=4'b0) && (load_stall_2==1'b1 || mem_en_2!=4'b0) && (pc_ID_2>pc_ID_1)) begin
          IFen_1 = 1'b0;
          IDen_1 = 1'b0;
          IDclr = 1'b0;
          IFen_2 = 1'b0;
          IDen_2 = 1'b0;
          EXen_2 = 1'b0;
        end
        if(load_stall_1==1'b1 && (pc_ID_2>pc_ID_1)) begin
          if((ctrl_we_2!=0) && (((regdst_2==1) && (dcd_rt_2!=0) && (dcd_rt_2==wr_reg_EX_1)) || ((dcd_rs_2!=0) && (dcd_rs_2==wr_reg_EX_1)))) begin
            IFen_2 = 1'b0;
            IDen_2 = 1'b0;
            EXen_2 = 1'b0;
            IFen_1 = 1'b0;
            IDen_1 = 1'b0;
            IDclr = 1'b0;
            CDen = 1'b1;
            CDAmt = 3'd1;
          end
        end
      end
    end
  end

endmodule
