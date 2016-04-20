module carry_select(a,b,cin,sum,co
 );
  input [3:0]a;
  input [3:0]b;
  input cin;
  output [3:0]sum;
  output co;
  wire [3:0]sum;
  wire co;
  wire s1,c1,s2,c2,s3,c3,s4,s11,s44,c4,c11,s22,c22,s33,c33,c44;

  /*always_comb begin
    $display("a: %x", a);
    $display("b: %x", b);
    $display("sum: %x", sum);
  end*/

  //assuming carry in 0
  fa x1(a[0],b[0],1'b0,s1,c1);
  fa x2(a[1],b[1],c1,s2,c2);
  fa x3(a[2],b[2],c2,s3,c3);
  fa x4(a[3],b[3],c3,s4,c4);
  //assuming carry in 1
  fa x5(a[0],b[0],1'b1,s11,c11);
  fa x6(a[1],b[1],c11,s22,c22);
  fa x7(a[2],b[2],c22,s33,c33);
  fa x8(a[3],b[3],c33,s44,c44);
  //select either carry 1 or 0 using carry out of FA
  //mux for sum select
  mux2to1 #(1) x9(s1,s11,cin,sum[0]);
  mux2to1 #(1) x10(s2,s22,cin,sum[1]);
  mux2to1 #(1) x11(s3,s33,cin,sum[2]);
  mux2to1 #(1) x12(s4,s44,cin,sum[3]);
  //mux for carry select
  mux2to1 #(1) x13(c4,c44,cin,co);
endmodule


module carry_select_n_bit(sum, in1, in2);
  parameter
            width = 32,
            blocksize = 7;

  output [width-1:0] sum;
  input [width-1:0] in1,in2;

  wire [width:0] carry;
  wire c1,c2,c3,c4;

  assign carry[0] = 1'b0;

  fa fa1(in1[0],in2[0],carry[0],sum[0],c1);
  fa fa2(in1[1],in2[1],carry[1],sum[1],c2);
  fa fa3(in1[2],in2[2],carry[2],sum[2],c3);
  fa fa4(in1[3],in2[3],carry[3],sum[3],c4);


  genvar i;
  generate
    for (i=1; i<=blocksize; i=i+1) begin
      carry_select cs(in1[4*(i+1)-1:4*i],in2[4*(i+1)-1:4*i],
                      carry[i-1],sum[4*(i+1)-1:4*i],carry[i]);
    end
  endgenerate

endmodule



