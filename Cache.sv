`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/26/2025 04:27:35 PM
// Design Name: 
// Module Name: Cache
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


module Cache(
    input [31:0] PC,
    input CLK,
    input update,
    input logic [31:0] w0,
    input logic [31:0] w1,
    input logic [31:0] w2, 
    input logic [31:0] w3,
    input logic [31:0] w4, 
    input logic [31:0] w5,
    input logic [31:0] w6, 
    input logic [31:0] w7,
    output logic [31:0] rd, 
    output logic hit, 
    output logic miss
);

//Parameters
parameter NUM_BLOCKS = 16;
parameter BLOCK_SIZE = 8;
parameter INDEX_SIZE = 4;
parameter WORD_OFFSET_SIZE = 3;
parameter BYTE_OFFSET = 2;
parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET;

//Internal Signals
logic [31:0] data[NUM_BLOCKS-1:0][BLOCK_SIZE-1:0];
logic [TAG_SIZE-1:0] tags[NUM_BLOCKS-1:0];
logic valid_bits[NUM_BLOCKS-1:0];
logic [3:0] index;
logic [TAG_SIZE-1:0]cache_tag, pc_tag;
logic [2:0] pc_offset;

//Initializing
initial begin
    int i;
    int j;
    for(i = 0; i < NUM_BLOCKS; i = i + 1) begin //initializing RAM to 0
        for(j=0; j < BLOCK_SIZE; j = j + 1)begin
        data[i][j] = 32'b0;
        end
        tags[i] = 32'b0;
        valid_bits[i] = 1'b0;
    end
end

//Adressing
assign index = PC[8:5];
assign validity = valid_bits[index];
assign cache_tag = tags[index];
assign pc_offset = PC[4:2];
assign pc_tag = PC[31:9];
assign hit = (validity && (cache_tag == pc_tag));
//always_ff @ (posedge CLK) begin
assign miss = !hit;
//end
//Read data
always_comb begin
    rd = 32'h00000013; //nop
    if(hit) begin
    rd = data[index][pc_offset];
    end
end

//Updating Cache
always_ff @(posedge CLK) begin
if(update) begin
    data[index][0] <= w0;
    data[index][1] <= w1;
    data[index][2] <= w2;
    data[index][3] <= w3;
    data[index][4] <= w4;
    data[index][5] <= w5;
    data[index][6] <= w6;
    data[index][7] <= w7;
    valid_bits[index] <= 1'b1;
    tags[index] <= pc_tag;
    end
end

endmodule