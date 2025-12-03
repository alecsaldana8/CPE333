`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/01/2024 09:23:10 AM
// Design Name: 
// Module Name: PC
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


module PC(
    input PC_RST,
    input PC_WE,
    input [31:0] PC_DIN,
    input CLK,
    output logic[31:0] PC_COUNT
    );
    always_ff @(posedge CLK) begin
        if(PC_RST) begin    //PC_Rst resets count to 0
            PC_COUNT<=0;
        end else if(PC_WE) begin    //otherwise move to next instruction specified by mux
            PC_COUNT<=PC_DIN;
        end else begin
            PC_COUNT<=PC_COUNT; //if write not enabled, and not reseting stay at same instruction
       end
    end
endmodule