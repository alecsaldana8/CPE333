`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/12/2024 09:29:48 AM
// Design Name: 
// Module Name: ALU
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


module ALU(
    input [31:0] srcA,
    input [31:0] srcB,
    input [3:0] alu_fun,
    output logic [31:0] alu_result
    );
    
    always_comb begin
        case(alu_fun) 
        4'b0000: alu_result = srcA+srcB;
        4'b1000: alu_result = srcA-srcB;
        4'b0110: alu_result = srcA|srcB;  //and, xor, srl, sll, sra, slt, sltu, lui-copy
        4'b0111: alu_result = srcA&srcB;
        4'b0100: alu_result = srcA^srcB;
        4'b0101: alu_result = srcA>>srcB[4:0];   //only want to shift the 5 least significant bits whenever shifting
        4'b0001: alu_result = srcA<<srcB[4:0];
        4'b1101: alu_result = $signed(srcA)>>>srcB[4:0];
        4'b0010: alu_result = $signed(srcA)<$signed(srcB);
        4'b0011: alu_result = srcA<srcB;
        default: alu_result = srcA; //will default to lui_copy, if not matching one of these for some reason
        endcase
    end
endmodule