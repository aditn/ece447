////
//// mux2to1
////
//// out (output) - data chosen to be outputted
//// in0 (input)  - data lines
//// in1 (input)  - data lines
//// sel (input)  - selects which data to output
////
module mux2to1 #(parameter width = 32) (
      output logic [width - 1:0] out,
      input logic [width - 1:0] in0, in1, 
      input logic sel);
    
    assign out = sel ? in1 : in0;

endmodule


////
//// mux4to1
////
//// out (output) - data chosen to be outputted
//// in0 (input)  - data lines
//// in1 (input)  - data lines
//// in2 (input)  - data lines
//// in3 (input)  - data lines
//// sel (input)  - selects which data to output
////
module mux4to1 #(parameter width = 32) (
      output logic [width - 1:0] out,
      input logic [width - 1:0] in0, in1, in2, in3,
      input logic [1:0] sel);

    assign out = sel[1] ? (sel[0] ? in3 : in2) : (sel[0] ? in1 : in0);

endmodule

////
//// mux5to1
////
//// out (output) - data chosen to be outputted
//// in0 (input)  - data lines
//// in1 (input)  - data lines
//// in2 (input)  - data lines
//// in3 (input)  - data lines
//// in4 (input)  - data lines
//// sel (input)  - selects which data to output
////
module mux5to1 #(parameter width = 32) (
      output logic [width - 1:0] out,
      input logic [width - 1:0] in0, in1, in2, in3, in4,
      input logic [2:0] sel);

    assign out = sel[2] ? in4 : (sel[1] ? (sel[0] ? in3 : in2) : (sel[0] ? in1 : in0));

endmodule