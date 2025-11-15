`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 02/22/2024 04:48:57 PM
// Design Name:
// Module Name: ForwardingUnit
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

module ForwardingUnit(
    input [4:0] rs1,
    input [4:0] rs2,
    input rs1_used,
    input rs2_used,
    input rd_used,
    input rd_used_mem,
    input stalledA,
    input stalledB,
    input [4:0] ExRd,

    input [4:0] MemRd,
    input ExRegWrite,
    input MemRegWrite,
    input RDEN2,
    input [1:0] pcSource,
    output reg [1:0] FwrdMuxA,
    output reg [1:0] FwrdMuxB,
    output reg stall,
    output reg stallB,
    output reg stallA,
    output reg flush
);
// Why won't stalledA & stalledB take precedent, this means that FwrdMux wont be 3 :(
always_comb begin
FwrdMuxA = 2'b00; FwrdMuxB = 2'b00; stall = 1'b0; stallA= 1'b0; stallB=1'b0;
flush = 1'b0;
if (pcSource != 0) begin
flush = 1'b1;
end
//load stall
if (RDEN2 && rd_used && (((ExRd == rs1) && rs1_used))) begin
stall = 1'b1;
stallA = 1'b1;
end
if (RDEN2 && rd_used && (((ExRd == rs2)&& rs2_used))) begin
stall = 1'b1;
stallB = 1'b1;
end
//forawrd
if (stalledA) begin
FwrdMuxA = 2'b11;
end
else if ((ExRd == rs1) && rs1_used && ExRegWrite && rd_used) begin
FwrdMuxA = 2'b01;
end
else if ((MemRd == rs1) && rs1_used && MemRegWrite && rd_used_mem) begin
FwrdMuxA = 2'b10;
end
if (stalledB) begin
FwrdMuxB = 2'b11;
end
else if ((ExRd == rs2) && rs2_used && ExRegWrite && rd_used) begin
FwrdMuxB = 2'b01;
end
else if ((MemRd == rs2) && rs2_used && MemRegWrite && rd_used_mem) begin
FwrdMuxB = 2'b10;
end
end
endmodule