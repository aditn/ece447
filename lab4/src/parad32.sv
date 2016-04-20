module parad32(a,cout,p,q);
  output [31:0]a;
  output cout;
  input [31:0]p,q;
  wire [6:0]c;
  
  parad4 u1(a[3:0],c[0],p[3:0],q[3:0]);
  parad4_full u2(a[7:4],c[1],p[7:4],q[7:4],c[0]);
  parad4_full u3(a[11:8],c[2],p[11:8],q[11:8],c[1]);
  parad4_full u4(a[15:12],c[3],p[15:12],q[15:12],c[2]);
  parad4_full u5(a[19:16],c[4],p[19:16],q[19:16],c[3]);
  parad4_full u6(a[23:20],c[5],p[23:20],q[23:20],c[4]);
  parad4_full u7(a[27:24],c[6],p[27:24],q[27:24],c[5]);
  parad4_full u8(a[31:28],cout,p[31:28],q[31:28],c[6]);
  
endmodule

module parad4_full(a,c,p,q,cin);
  output [3:0] a;
  output c;
  input cin;
  input [3:0]p,q;

  fa u1(cin,p[0],q[0], a[0],c1);
  fa u2(c1,p[1],q[1],a[1],c2);
  fa u3(c2,p[2],q[2],a[2],c3);
  fa u4(c3,p[3],q[3],a[3],c);

endmodule

module parad4(a,c,p,q);
    output [3:0]a;
    output c;
    input [3:0]p,q;
    wire c1,c2,c3;

    ha u1(p[0],q[0],a[0],c1);
    fa u2(c1,p[1],q[1],a[1],c2);
    fa u3(c2,p[2],q[2],a[2],c3);
    fa u4(c3,p[3],q[3],a[3],c);
endmodule


module fa(a,b,c,sum,carry);
  input a,b,c;
  output sum,carry;
  //wire sum,carry;
  assign sum=(a^b^c); // sum bit
  assign carry=((a&b) | (b&c) | (a&c)); //carry bit
endmodule

module ha(a,b,sum,carry);
  input a,b;
  output sum, carry;
  //wire sum, carry;
  assign sum = a^b; // sum bit
  assign carry = (a&b) ;
  //carry bit
endmodule

