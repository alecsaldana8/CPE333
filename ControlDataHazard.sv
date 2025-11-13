`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 08/02/2025 10:37:49 AM
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


module DataHazardUnit(
    input logic [6:0] opcode,
    input logic [4:0] de_adr1,
    input logic [4:0] de_adr2,
    input logic [4:0] ex_adr1,
    input logic [4:0] ex_adr2,
    input logic [4:0] ex_rd,
    input logic [4:0] mem_rd,
    input logic [4:0] wb_rd,
    input logic ex_regWrite,
    input logic mem_regWrite,
    input logic wb_regWrite,
    input logic de_rs1_used,
    input logic de_rs2_used,
    input logic ex_rs1_used,
    input logic ex_rs2_used,
    input logic [31:0] ex_mem_IR,
    input logic [31:0] de_ex_IR,
    input logic [31:0] de_ex_aluRes,
    
    output logic [1:0] fsel1,
    output logic [1:0] fsel2,
    output logic load_use_haz
    
    );
    logic ex_is_load;
    logic mem_is_jal;
    
    always_comb begin
        fsel1=2'b00;
        fsel2=2'b00;
        load_use_haz=1'b0;

        
        
        //forwaridng from ex stage
       if (ex_regWrite && (ex_rd!= 5'd0)) begin           // ignore x0
            if (de_rs1_used && (ex_rd == de_adr1)) fsel1 = 2'b01;
            if (de_rs2_used && (ex_rd == de_adr2)) fsel2 = 2'b01;
        end
        
        mem_is_jal = (ex_mem_IR[6:0] == 7'b1101111);//load instruction opcode

        
       //forwarding from mem stage
       if (mem_regWrite && (mem_rd != 5'd0)) begin
            if (mem_is_jal && (de_rs2_used && (mem_rd == de_adr2) && (fsel2 == 2'b00))) fsel2 = 2'b11;//handles jal data hazard
           if (de_rs1_used && (mem_rd == de_adr1) && (fsel1 == 2'b00)) fsel1 = 2'b10;
           if (de_rs2_used && (mem_rd == de_adr2) && (fsel2 == 2'b00)) fsel2 = 2'b10;
       end   
       
       
    
    //load use hazard
        ex_is_load = (de_ex_IR[6:0] == 7'b0000011);//load instruction opcode

        if ((ex_is_load = (de_ex_IR[6:0] == 7'b0000011)) && (
            (de_rs1_used && (ex_rd == de_adr1)) ||
            (de_rs2_used && (ex_rd == de_adr2))
        )) begin
            load_use_haz = 1'b1;   // signal the control logic to stall ID and IF for 1 cycle
        end
        end
        
  
endmodule

