module carry_select(a,b,cin,sum,co);
  input [31:0]a;
  input [31:0]b;
  input cin;
  output [31:0]sum;
  output co;
  //wire [3:0]sum;
  //wire co;
  
  //wire s1,c1,s2,c2,s3,c3,s4,s11,s44,c4,c11,s22,c22,s33,c33,c44;
  //assuming carry in 0
  /*fa x1(a[0],b[0],0,s_0[0],c_0[0]);
  fa x2(a[1],b[1],c_0[0],s_0[1],c_0[1]);
  fa x3(a[2],b[2],c_0[1],s_0[2],c_0[2]);
  fa x4(a[3],b[3],c_0[2],s_0[3],c_0[3]);
  //assuming carry in 1
  fa x5(a[0],b[0],1,s_1[0],c_1[0]);
  fa x6(a[1],b[1],c_1[0],s_1[1],c_1[1]);
  fa x7(a[2],b[2],c_1[1],s_1[2],c_1[2]);
  fa x8(a[3],b[3],c_1[2],s_1[3],c_1[3]);*/


  genvar i;
  wire [31:0] s_0,s_1, c_0, c_1;
  generate 
    for (i=0; i<=31; i= i+1) begin
      if (i==0) begin 
        fa f_0(a[i],b[i],1'b0,s_0[i],c_0[i]);
        fa f_1(a[i],b[i],1'b1,s_1[i],c_1[i]);
        mux x9(s_0[i],s_1[i],cin,sum[i]);
      end
      else if (i == 31) begin
        mux x13(c_0[i],c_1[i],cin,co);
        fa f_0(a[i],b[i],c_0[i-1],s_0[i],c_0[i]);
        fa f_1(a[i],b[i],c_1[i-1],s_1[i],c_1[i]);
        mux x10(s_0[i],s_1[i],cin,sum[i]);
      end
      else begin
        fa f_0(a[i],b[i],c_0[i-1],s_0[i],c_0[i]);
        fa f_1(a[i],b[i],c_1[i-1],s_1[i],c_1[i]);
        mux x10(s_0[i],s_1[i],cin,sum[i]);
      end
    end
  endgenerate



  //select either carry 1 or 0 using carry out of FA
  //mux for sum select
  
  //mux x10(s_0[1],s_1[1],cin,sum[1]);
  //mux x11(s_0[2],s_1[2],cin,sum[2]);
  //mux x12(s_0[3],s_1[3],cin,sum[3]);
  //mux for carry select
  
endmodule

module mux(a,b,s,q);
  input a;
  input b;
  input s;
  output q;
  //wire q;
  assign q=s?b:a;
endmodule