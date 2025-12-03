`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date:
// Design Name:
// Module Name:
// Project Name:
// Target Devices:
// Description:
//////////////////////////////////////////////////////////////////////////////////


module CU_DCDR(
    input logic [6:0] ir_opcode_dcdr,
    input logic [2:0] ir_funct_dcdr,
    input logic ir_30_dcdr,
    input logic int_taken_dcdr,

    output logic jump,
    output logic branch,
    output logic store,
    output logic regWrite,
    output logic memWE2,
    output logic memRDEN2,
    output logic [3:0] alu_fun_dcdr,
    output logic [1:0] alu_scra_dcdr,
    output logic [2:0] alu_scrb_dcdr,
    output logic [1:0] rf_wr_sel_dcdr
    );
   
    always_comb begin
    // Initialize all outputs to zeros
        alu_fun_dcdr = 4'b0000;
        alu_scra_dcdr = 2'b00;
        alu_scrb_dcdr = 3'b000;
        rf_wr_sel_dcdr = 2'b00;
        regWrite = 1'b0;
        memWE2 = 1'b0;
        memRDEN2 = 1'b0;  
        jump = 1'b0;
        branch = 1'b0;
        store = 0;
       
        case (ir_opcode_dcdr)
            // R-type    
            7'b0110011: begin // (add, and, or, sll, sly, slty, sra, srl, sub, xor)
                alu_fun_dcdr = {ir_30_dcdr, ir_funct_dcdr};
                alu_scra_dcdr = 2'b00;
                alu_scrb_dcdr = 3'b000;
                rf_wr_sel_dcdr = 2'b11;  
                regWrite = 1'b1;          
            end
            // I-type
            7'b0010011: begin // (addi, ori, slli, slti, sltiu, xori)

                if (ir_funct_dcdr == 3'b101) // srai and srli
                    alu_fun_dcdr = {ir_30_dcdr, ir_funct_dcdr};
                else
                    alu_fun_dcdr = {1'b0, ir_funct_dcdr};                    
                alu_scra_dcdr = 2'b00;
                alu_scrb_dcdr = 3'b001;
                rf_wr_sel_dcdr = 2'b11;
                regWrite = 1'b1;
            end
             
            // I-type - jalr case          
            7'b1100111: begin
                rf_wr_sel_dcdr = 2'b00;
                regWrite = 1'b1;
                jump = 1'b1;
            end
                               
            // I-type - load case
            7'b0000011: begin // (lb, lbu, lh, lhu, lw)
                alu_fun_dcdr = 4'b0000;
                alu_scra_dcdr = 2'b00;
                alu_scrb_dcdr = 3'b001;
                rf_wr_sel_dcdr = 2'b10;
                regWrite = 1'b1;
                memRDEN2 = 1'b1;
                memWE2 = 1'b0;
            end
                                                                                                         
            // S-type
            7'b0100011: begin //sb, sh, sw
                alu_fun_dcdr = 4'b0000;
                alu_scra_dcdr = 2'b00;
                alu_scrb_dcdr = 3'b010;
                memRDEN2 = 1'b1;
                memWE2 = 1'b1;
                store = 1;
            end
                                         
            // B-type
            7'b1100011: begin
                regWrite = 1'b0;
                memRDEN2 = 1'b0;
                memWE2 = 1'b0;
                branch = 1'b1;        
            end
              // J-type
            7'b1101111: begin // jal
                rf_wr_sel_dcdr = 2'b00;
                regWrite = 1'b1;
                jump = 1'b1;
                end  
                // U-type
            7'b0110111: begin //lui
                alu_fun_dcdr = 4'b1001;
                alu_scra_dcdr = 2'b01;
                rf_wr_sel_dcdr = 2'b11;
                regWrite = 1'b1;
                end                    
            7'b0010111: begin //auipc
                alu_fun_dcdr = 4'b0000;
                alu_scra_dcdr = 2'b01;
                alu_scrb_dcdr = 3'b011;
                rf_wr_sel_dcdr = 2'b11;
                regWrite = 1'b1;
                end  
                   
            // I-type - jalr case          
            7'b1100111: begin
                rf_wr_sel_dcdr = 2'b00;
                regWrite = 1'b1;
                jump = 1'b1;
            end
                             
 
        endcase
    end                    
endmodule