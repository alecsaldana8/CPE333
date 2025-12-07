`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/26/2025 04:30:21 PM
// Design Name: 
// Module Name: imem
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module imem(
 input  logic [31:0] a,
 output logic [31:0] w0,
 output logic [31:0] w1,
 output logic [31:0] w2,
 output logic [31:0] w3,
 output logic [31:0] w4,
 output logic [31:0] w5,
 output logic [31:0] w6,
 output logic [31:0] w7
);

//mem array
 logic [31:0] ram[0:16383];
 logic [29:0] actual_address;
 
 assign actual_address = {a[31:5], 3'b000};
 
 // Initialize mem from file
 initial $readmemh("test_All.mem", ram, 0, 16383);
 
 //changed memory so it does output 8 words
 assign w0 = ram[actual_address];
 assign w1 = ram[actual_address+1];
 assign w2 = ram[actual_address+2];
 assign w3 = ram[actual_address+3];
 assign w4 = ram[actual_address+4];
 assign w5 = ram[actual_address+5];
 assign w6 = ram[actual_address+6];
 assign w7 = ram[actual_address+7];
endmodule