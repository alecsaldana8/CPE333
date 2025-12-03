`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/09/2024 05:58:40 PM
// Design Name: 
// Module Name: RegFile
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


module RegFile(
    input w_en,
    input [4:0] adr1,
    input [4:0] adr2,
    input [4:0] w_adr,
    input [31:0] w_data,
    input clk,
    output logic [31:0] rs1,
    output logic[31:0] rs2
    );
   
   
    // Create a memory module with 16-bit width and 512 addresses
logic [31:0] ram [0:31];
// Initialize the memory to be all 0s
initial begin
  int i;
  for (i=0; i<32; i=i+1) begin
    ram[i] = 0;
  end
end

// Access the ram memory by address
// ram[address]    // ram[100] = data stored in address 100

always_comb begin
    if(adr1==0)
        rs1=0;
    else
        rs1=ram[adr1];
        
    if(adr2==0)
        rs2=0;
    else
        rs2=ram[adr2];
end

always_ff @(negedge clk) begin
    if(w_en && w_adr != 0) 
       ram[w_adr]<=w_data; 
end

endmodule