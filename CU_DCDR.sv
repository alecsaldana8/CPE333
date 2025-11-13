
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Alec Saldana
// Create Date: 
// Module Name: CU_DCDR (updated for pipeline)
//////////////////////////////////////////////////////////////////////////////////

module CU_DCDR(
    input  logic        IR_30,
    input  logic [6:0]  IR_OPCODE,
    input  logic [2:0]  IR_FUNCT,
    input  logic        BR_EQ,
    input  logic        BR_LT,
    input  logic        BR_LTU,
    output logic [3:0]  ALU_FUN,
    output logic        ALU_SRCA,
    output logic [1:0]  ALU_SRCB,
    output logic [2:0]  PC_SOURCE,
    output logic [1:0]  RF_WR_SEL,
    output logic        REG_WRITE,
    output logic        MEM_WRITE,
    output logic        MEM_READ2
);

    // Combinational control logic
    always_comb begin
        // Default values for all outputs
        ALU_FUN     = 4'b0000;
        ALU_SRCA    = 1'b0;
        ALU_SRCB    = 2'b00;
        PC_SOURCE   = 3'b000;
        RF_WR_SEL   = 2'b00;
        REG_WRITE   = 1'b0;
        MEM_WRITE   = 1'b0;
        MEM_READ2   = 1'b0;

        case (IR_OPCODE)
            7'b0010111: begin // AUIPC
                ALU_SRCA    = 1'b1;    // Use PC
                ALU_SRCB    = 2'b11;   // U-type immediate
                RF_WR_SEL   = 2'b11;   // ALU result
                REG_WRITE   = 1'b1;
            end

            7'b0110111: begin // LUI
                ALU_FUN     = 4'b1001; // LUI special code (treated like pass U-immed)
                ALU_SRCA    = 1'b1;
                ALU_SRCB    = 2'b11;
                RF_WR_SEL   = 2'b11;
                REG_WRITE   = 1'b1;
            end

            7'b1101111: begin // JAL
                PC_SOURCE   = 3'b011;
                RF_WR_SEL   = 2'b00;   // Write PC+4 to rd
                REG_WRITE   = 1'b1;
            end

            7'b1100111: begin // JALR
              PC_SOURCE   = 3'b001;
                RF_WR_SEL   = 2'b00;
                REG_WRITE   = 1'b1;
                ALU_SRCB    = 2'b01;   // I-type
            end

            7'b1100011: begin // BRANCH
                case (IR_FUNCT)
                    3'b000: PC_SOURCE = (BR_EQ)  ? 3'b010 : 3'b000; // BEQ
                   3'b001: PC_SOURCE = (~BR_EQ) ? 3'b010 : 3'b000; // BNE
                    3'b100: PC_SOURCE = (BR_LT)  ? 3'b010 : 3'b000; // BLT
                    3'b101: PC_SOURCE = (~BR_LT) ? 3'b010 : 3'b000; // BGE
                    3'b110: PC_SOURCE = (BR_LTU) ? 3'b010 : 3'b000; // BLTU
                   3'b111: PC_SOURCE = (~BR_LTU)? 3'b010 : 3'b000; // BGEU
                    default: PC_SOURCE = 3'b000;
                endcase
 
            end

            7'b0000011: begin // LOAD
                ALU_SRCB    = 2'b01;   // I-type
                ALU_FUN     = 4'b0000; // ADD
                RF_WR_SEL   = 2'b10;   // Mem output
                REG_WRITE   = 1'b1;
                MEM_READ2   = 1'b1;
            end

            7'b0100011: begin // STORE
                ALU_SRCB    = 2'b10;   // S-type
                ALU_FUN     = 4'b0000; // ADD
                MEM_WRITE   = 1'b1;
                REG_WRITE = 1'b0;
            end

            7'b0010011: begin // I-Type ALU (e.g., addi, slli, etc.)
                ALU_SRCB    = 2'b01;   // I-type immediate
                RF_WR_SEL   = 2'b11;   // ALU result
                REG_WRITE   = 1'b1;

                case (IR_FUNCT)
                    3'b000: ALU_FUN = 4'b0000; // ADDI
                    3'b001: ALU_FUN = 4'b0001; // SLLI
                    3'b010: ALU_FUN = 4'b0010; // SLTI
                    3'b011: ALU_FUN = 4'b0011; // SLTIU
                    3'b100: ALU_FUN = 4'b0100; // XORI
                    3'b101: ALU_FUN = (IR_30) ? 4'b1101 : 4'b0101; // SRAI or SRLI
                    3'b110: ALU_FUN = 4'b0110; // ORI
                    3'b111: ALU_FUN = 4'b0111; // ANDI
                endcase
            end

            7'b0110011: begin // R-Type ALU (e.g., add, sub, etc.)
                ALU_FUN     = {IR_30, IR_FUNCT}; // 4-bit ALU control
                RF_WR_SEL   = 2'b11;
                REG_WRITE   = 1'b1;
            end

            7'b1110011: begin // SYSTEM (e.g., CSRRS)
                ALU_SRCB    = 2'b01;
                ALU_SRCA    = 1'b0;
                RF_WR_SEL   = 2'b01; // CSR reg
                REG_WRITE   = 1'b1;
            end

            default: begin
                // All outputs are already defaulted to safe values
            end
        endcase
    end

endmodule
