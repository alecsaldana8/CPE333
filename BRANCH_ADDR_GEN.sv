`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/15/2024 11:41:22 AM
// Design Name: 
// Module Name: BRANCH_ADDR_GEN
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

module BRANCH_ADDR_GEN(
    input [31:0] J_TYPE_IMM,
    input [31:0] B_TYPE_IMM,
    input [31:0] I_TYPE_IMM,
    input [31:0] rs1,
    input [31:0] PC_B,
    input [31:0] PC_J,
    output [31:0] branch,
    output [31:0] jal,
    output [31:0] jalr
    );
    assign branch = PC_B + B_TYPE_IMM;
    assign jal = PC_J + J_TYPE_IMM;
    assign jalr = rs1 + I_TYPE_IMM;
    
endmodule