`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/18/2025 10:00:13 PM
// Design Name: 
// Module Name: HazardUnit
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


module HazardUnit(
    input logic [4:0] rs1_in,
    input logic [4:0] rs2_in,
    input logic [4:0] de_rd,
    input logic de_rd_reg_write,
    input logic [4:0] ex_rd,
    input logic ex_rd_reg_write,
    input logic [4:0] mem_rd,
    input logic mem_rd_reg_write,
    input logic [4:0] de_rs1_jalr,
    input logic load, // Checked in ex
    input logic de_ex_load,
    input logic jal_taken,
    input logic jalr_taken,
    input logic branch,
    input logic store,
    input logic imm_A,
    input logic imm_B,
    input logic de_ex_stalled,
    output logic [1:0] pc_mux_out,
    output logic [1:0] jalr_sel,
    output logic [1:0] forward_rs2,
    output logic flush_if_de,
    output logic flush_de_ex,
    output logic stall,
    output logic [1:0] alu_sel_1,
    output logic [1:0] alu_sel_2
    
    );
    
    always_comb begin
    // Init outputs
    alu_sel_1 = 0;
    alu_sel_2 = 0;
    flush_if_de = 0;
    flush_de_ex = 0;
    stall = 0;
    forward_rs2 = 0;
    jalr_sel = 0;
    
    
    if (1) begin   // Cannot override x0, so no hazard if rd is 0
    
    // Check one instruction above
        if (rs1_in == ex_rd && ex_rd != 0 && (ex_rd_reg_write || store) && !imm_A && !de_ex_stalled) begin // RAW 1 instruction above
            alu_sel_1 = 1;
            if (load) stall = 1;
            // Check 2 instructions above
        end else if (rs1_in == mem_rd && mem_rd_reg_write && mem_rd != 0 && !imm_A && alu_sel_1 != 1) alu_sel_1 = 2; 
        
        
        if (rs2_in == ex_rd && (ex_rd_reg_write || store) && ex_rd != 0 && (!imm_B || store) && !de_ex_stalled) begin
            forward_rs2 = 1;
            if (!imm_B) begin
                alu_sel_2 = 1;
                if (load) stall = 1;
            end
            // Check 2 instructions above
        end else if (rs2_in == mem_rd && (mem_rd_reg_write || store) && mem_rd != 0 && alu_sel_2 != 1) begin
            forward_rs2 = 2;
            if (!imm_B) begin
                alu_sel_2 = 2;
            end
        end
        
        
    
        
        
           
    end
    
    // Check forwarding for jalr
    if (de_rs1_jalr == de_rd && de_rd_reg_write) begin
        jalr_sel = 1;
        if (de_ex_load && jalr_taken) stall = 1;
    end
    else if (de_rs1_jalr == ex_rd && ex_rd_reg_write) begin
        jalr_sel = 2; 
        if (load && jalr_taken) stall = 1;
    end
    else if (de_rs1_jalr == mem_rd && mem_rd_reg_write) jalr_sel = 3;
    else jalr_sel = 0;
    
    if (branch) pc_mux_out = 2;
    else if (jal_taken) pc_mux_out = 3;
    else if (jalr_taken) pc_mux_out = 1;
    else if (jal_taken || jalr_taken) flush_if_de = 1;
    else pc_mux_out = 0;
    
    if (branch) begin
        flush_if_de = 1; 
        flush_de_ex = 1;
    end else if (jal_taken || jalr_taken) flush_if_de = 1;
     
    end
endmodule