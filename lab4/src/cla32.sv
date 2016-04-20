module cla32( d1,d2,clk,cin,sum,cout); //32 bit CLA using 8 4-bit CLA adderes
    input [31:0] d1;
    input [31:0] d2;
    input clk;
    input cin;
    output cout;
    output [31:0] sum;

  wire c0,c1,c2,c3,c4,c5,c6;
  reg [31:0] b;
  always@(posedge clk)
  begin
    if(cin==1)
      b<=-d2-1;
    else
      b<=d2;
  end
  
  cla4 n1(d1[3:0],b[3:0],cin,sum[3:0],c0);
  cla4 n2(d1[7:4],b[7:4],c0,sum[7:4],c1);
  cla4 n3(d1[11:8],b[11:8],c1,sum[11:8],c2);
  cla4 n4(d1[15:12],b[15:12],c2,sum[15:12],c3);
  cla4 n5(d1[19:16],b[19:16],c3,sum[19:16],c4);
  cla4 n6(d1[23:20],b[23:20],c4,sum[23:20],c5);
  cla4 n7(d1[27:24],b[27:24],c5,sum[27:24],c6);
  cla4 n8(d1[31:28],b[31:28],c6,sum[31:28],cout);
 endmodule

//4 Bit CLA code

module cla4(a,b,cin,s,cout);
input[3:0] a,b;
input cin;
output cout;
output[3:0] s;
wire[3:0] g,p;
wire[13:0] z;
  xor21 x1 (.a1(a[0]),.a2(b[0]),.z(p[0]));
  and21 x2 (.a1(a[0]),.a2(b[0]),.z(g[0]));
  xor21 x3 (.a1(a[1]),.a2(b[1]),.z(p[1]));
  and21 x4 (.a1(a[1]),.a2(b[1]),.z(g[1]));
  xor21 x5 (.a1(a[2]),.a2(b[2]),.z(p[2]));
  and21 x6 (.a1(a[2]),.a2(b[2]),.z(g[2]));
  xor21 x7 (.a1(a[3]),.a2(b[3]),.z(p[3]));
  and21 x8 (.a1(a[3]),.a2(b[3]),.z(g[3]));
  xor21 x9 (.a1(cin),.a2(p[0]),.z(s[0]));
  and21 x10 (.a1(cin),.a2(p[0]),.z(z[0]));
  or21 x11 (.a1(z[0]),.a2(g[0]),.z(z[1]));
  xor21 x12 (.a1(z[1]),.a2(p[1]),.z(s[1]));
  and31 x13 (.a1(cin),.a2(p[0]),.a3(p[1]),.z(z[2]));
  and21 x14 (.a1(g[0]),.a2(p[1]),.z(z[3]));
  or31 x15 (.a1(z[2]),.a2(z[3]),.a3(g[1]),.z(z[4]));
  xor21 x16 (.a1(z[4]),.a2(p[2]),.z(s[2]));
  and41 x17 (.a1(cin),.a2(p[0]),.a3(p[1]),.a4(p[2]),.z(z[5]));
  and31 x18 (.a1(g[0]),.a2(p[1]),.a3(p[2]),.z(z[6]));
  and21 x19 (.a1(g[1]),.a2(p[2]),.z(z[7]));
  or41 x20 (.a1(z[5]),.a2(z[6]),.a3(z[7]),.a4(g[2]),.z(z[8]));
  xor21 x21 (.a1(z[8]),.a2(p[3]),.z(s[3]));
  and41 x22 (.a1(cin),.a2(p[0]),.a3(p[1]),.a4(p[2]),.z(z[9]));
  and31 x23 (.a1(g[0]),.a2(p[1]),.a3(p[2]),.z(z[10]));
  and21 x24 (.a1(g[1]),.a2(p[2]),.z(z[11]));
  or41 x25 (.a1(z[9]),.a2(z[10]),.a3(z[11]),.a4(g[2]),.z(z[12]));
  and21 x26 (.a1(z[12]),.a2(p[3]),.z(z[13]));
  or21 x27 (.a1(z[13]),.a2(g[3]),.z(cout));
endmodule