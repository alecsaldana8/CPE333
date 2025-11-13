`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/10/2025 01:36:10 PM
// Design Name: 
// Module Name: ControlHazardUnit
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

module ControlHazardUnit(
    input CLK,
    input [6:0] de_opcode,
    input [6:0] ex_opcode,
    input [2:0] ex_func3,
    input  logic BR_EQ,
    input  logic BR_LT,
    input  logic BR_LTU,
    input [31:0] ex_I_Immed,//jalr
    input [31:0] ex_rs1,//jalr
    input [31:0] ex_pc,//branch
    input [31:0] ex_B_Immed,//branch
    input [31:0] de_pc,//jal
    input [31:0] de_J_Immed,//jal
    input logic branch_flag,
    output logic [31:0] jalr_pc,
    output logic [31:0] branch_pc,
    output logic [31:0] jal_pc,
    output logic [1:0] pc_sel,
    output logic flush,
    output logic de_flush
    );
    logic        branch_taken;
    always_comb begin
        unique case (ex_func3)
          3'b000: branch_taken = (ex_opcode==7'b1100011) &&  BR_EQ;   // BEQ
          3'b001: branch_taken = (ex_opcode==7'b1100011) && !BR_EQ;   // BNE
          3'b100: branch_taken = (ex_opcode==7'b1100011) &&  BR_LT;   // BLT
          3'b101: branch_taken = (ex_opcode==7'b1100011) && !BR_LT;   // BGE
          3'b110: branch_taken = (ex_opcode==7'b1100011) &&  BR_LTU;  // BLTU
          3'b111: branch_taken = (ex_opcode==7'b1100011) && !BR_LTU;  // BGEU
          default: branch_taken = 1'b0;
        endcase
    end
    
    always_ff @(negedge CLK) begin
    
            pc_sel<=2'b00;
            flush<=0;
            de_flush<=0;


    branch_pc<=ex_pc+$signed(ex_B_Immed);
    jal_pc<= de_pc+$signed(de_J_Immed);
    jalr_pc<= ex_rs1+$signed(ex_I_Immed);
    
    
    if(de_opcode==7'b1101111) begin//jal control hazard
        pc_sel<=2'b11;
        flush<=1;
    end


    if(ex_opcode == 7'b1100111 && !branch_flag ) begin//jalr control hazard
        pc_sel<=2'b01;
        flush<=1;
        de_flush<=1;
    end else if (ex_opcode==7'b1100011 && branch_taken   && !branch_flag) begin     //branch control hazard
                pc_sel<=2'b10;
                 flush<=1;
                 de_flush<=1;
            //end
        end


    end    

    
    
endmodule
