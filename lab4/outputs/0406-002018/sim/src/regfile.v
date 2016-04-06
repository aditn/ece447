/*
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2016 James C. Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 */

module regfile2_forward (/*AUTOARG*/
   // Outputs
   p_rs_data, p_rt_data,
   s_rs_data, s_rt_data,
   // Inputs
   p_rs_num, p_rt_num, p_rd_num, p_rd_data, p_rd_we, 
   s_rs_num, s_rt_num, s_rd_num, s_rd_data, s_rd_we, 
   clk, rst_b, halted
   );
	input       [4:0]  p_rs_num, p_rt_num, p_rd_num; 
	input       [31:0] p_rd_data;
	input              p_rd_we;
	output wire [31:0] p_rs_data, p_rt_data;

   	input       [4:0]  s_rs_num, s_rt_num, s_rd_num; 
	input       [31:0] s_rd_data;
	input              s_rd_we;
	output wire [31:0] s_rs_data, s_rt_data;

        input 		   clk, rst_b, halted;

	reg         [31:0] mem[0:31];
	integer            i;

	always @(posedge clk or negedge rst_b) begin 
		if (!rst_b) begin
			`include "regfile_init.vh"
		end else begin
		   if (p_rd_we && (p_rd_num != 0) && !(s_rd_we&&(p_rd_num==s_rd_num))) begin 
			mem[p_rd_num] <= p_rd_data;
		   end   
		   if (s_rd_we && (s_rd_num != 0)) begin 
			mem[s_rd_num] <= s_rd_data;
		   end   
		end 
	end 

	assign p_rs_data = (  (p_rs_num == 0) ? 
                              32'h0 : 
                              (  (s_rd_we&&(s_rd_num==p_rs_num)) ? 
                                 s_rd_data :
                                 (  (p_rd_we&&(p_rd_num==p_rs_num)) ?
                                     p_rd_data :
                                     mem[p_rs_num] )  )  );

	assign p_rt_data = (  (p_rt_num == 0) ? 
                              32'h0 : 
                              (  (s_rd_we&&(s_rd_num==p_rt_num)) ? 
                                 s_rd_data :
                                 (  (p_rd_we&&(p_rd_num==p_rt_num)) ?
                                     p_rd_data :
                                     mem[p_rt_num] )  )  );

	assign s_rs_data = (  (s_rs_num == 0) ? 
                              32'h0 : 
                              (  (s_rd_we&&(s_rd_num==s_rs_num)) ? 
                                 s_rd_data :
                                 (  (p_rd_we&&(p_rd_num==s_rs_num)) ?
                                     p_rd_data :
                                     mem[s_rs_num] )  )  );

	assign s_rt_data = (  (s_rt_num == 0) ? 
                              32'h0 : 
                              (  (s_rd_we&&(s_rd_num==s_rt_num)) ? 
                                 s_rd_data :
                                 (  (p_rd_we&&(p_rd_num==s_rt_num)) ?
                                     p_rd_data :
                                     mem[s_rt_num] )  )  );
			     
	// synthesis translate_off
	integer fd;
	always @(halted) begin
		if (rst_b && halted) begin
			fd = $fopen("regdump.txt");

			$display("--- 18-447 Register file dump ---");
			$display("=== Simulation Cycle %d ===", $time);
			
			$fdisplay(fd, "--- 18-447 Register file dump ---");
			$fdisplay(fd, "=== Simulation Cycle %d ===", $time);
			
			for(i = 0; i < 32; i = i+1) begin
				$display("R%d\t= 0x%8x\t( %0d )", i, mem[i], mem[i]);
				$fdisplay(fd, "R%d\t= 0x%8h\t( %0d )", i, mem[i], mem[i]); 
			end
			
			$display("--- End register file dump ---");
			$fdisplay(fd, "--- End register file dump ---");
			
			$fclose(fd);
		end
	end
	// synthesis translate_on

endmodule

//
// SRAM of size WIDTHx(2^SIZE_LG2)
//
// 1 combinational read port, 1 synchronous write port
//
// Works the same as the register file without forwarding
//
module btbsram #(parameter WIDTH=62, SIZE_LG2=7) (
   // Outputs
   rd_data,
   // Inputs
   rd_idx, wr_idx, wr_data, wr_we, clk, rst_b
   );
	input       [SIZE_LG2-1:0]  rd_idx, wr_idx; 
	input       [WIDTH-1:0] wr_data;
	input              wr_we, clk, rst_b ;

	output wire [WIDTH-1:0] rd_data ;

	reg         [WIDTH-1:0] mem[0:(2**SIZE_LG2)-1];
	integer            i;

	always @(posedge clk or negedge rst_b) begin 
		if (!rst_b) begin
		   for (i = 0; i < (2**SIZE_LG2); i = i+1) begin
		     mem[i] <= 0;
		   end
		end else if (wr_we) begin 
			mem[wr_idx] <= wr_data; 
		end 
	end 

	assign rd_data = (!rst_b)? 0 : mem[rd_idx];
endmodule


