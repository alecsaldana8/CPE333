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

