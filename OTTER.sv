`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 03/02/2023 04:17:51 PM
// Module Name: OTTER
//////////////////////////////////////////////////////////////////////////////////

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [2:0] funct3;    // Store funct3 for branches
    logic [31:0] pc;
} instr_t;

module OTTER(input CLK,
              input RST,
              input [31:0] IOBUS_IN,
              output [31:0] IOBUS_OUT,
              output [31:0] IOBUS_ADDR,
              output logic IOBUS_WR 
);           
    // pipeline-specific signals
    wire [6:0] opcode;
    wire [31:0] pc, aluResult, mem_data;
    
    // Additional signal declarations
    logic [13:0] imem_addr;
    
    // Signal declarations for pipeline (avoid conflicts with struct fields)
    logic pc_write_enable, reg_write_enable, mem_read_enable;
    logic mem_write_enable, mem_read2_enable;  // Memory control signals
    logic [1:0] mem_size;
    logic [3:0] alu_function;
    
    logic br_lt, br_eq, br_ltu;
    
    wire [31:0] IR;
    wire memRead1;
    
    // Memory instantiation
    Memory OTTER_MEMORY (
        .MEM_CLK(CLK), 
        .MEM_RDEN1(memRead1), 
        .MEM_RDEN2(mem_read2_enable), 
        .MEM_WE2(mem_write_enable), 
        .MEM_ADDR1(imem_addr), 
        .MEM_ADDR2(IOBUS_ADDR), 
        .MEM_DIN2(IOBUS_OUT), 
        .MEM_SIZE(mem_size),
        .MEM_SIGN(ex_mem_inst.mem_type[2]),  // Sign bit from memory type
        .IO_IN(IOBUS_IN), 
        .IO_WR(IOBUS_WR), 
        .MEM_DOUT1(IR), 
        .MEM_DOUT2(mem_data)
    );
              
//==== Instruction Fetch ===========================================
    
    // Pipeline registers: IF/DE
    logic [31:0] if_de_pc, if_de_ir;
    
    // PC signals
    logic [2:0] pc_source;
    logic [31:0] jalr_target, branch_target, jal_target;
    logic [31:0] pc_plus_4;
    
    assign pc_write_enable = 1'b1; // Always write(no hazards)
    
    // PC Module
    PC OTTER_PC (
        .CLK(CLK),
        .RST(RST),
        .PC_WRITE(pc_write_enable),
        .PC_SOURCE(pc_source),
        .JALR(jalr_target),
        .BRANCH(branch_target),
        .JAL(jal_target),
        .MTVEC(32'b0),        
        .MEPC(32'b0),        
        .PC_OUT(pc),
        .PC_OUT_INC(pc_plus_4)
    );
    
    // Instruction Memory
    assign imem_addr = pc[15:2];  // Word-aligned instruction address
    assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
    
    // IF/DE Pipeline Register
    always_ff @(posedge CLK) begin
        if (RST) begin
            if_de_pc <= 32'b0;
            if_de_ir <= 32'b0;
        end else begin
            if_de_pc <= pc;
            if_de_ir <= IR;  // IR comes from memory
        end
    end
     





     
//==== Instruction Decode ===========================================
    
    // DE/EX Pipeline registers
    logic [31:0] de_ex_opA, de_ex_opB, de_ex_rs2, de_ex_pc;
    logic [31:0] de_ex_i_imm, de_ex_b_imm, de_ex_j_imm;  
    instr_t de_ex_inst, de_inst;
    
    // Instruction decoding
    logic [6:0] de_opcode;
    logic [2:0] de_funct3;
    logic de_funct7_30;
    
    assign de_opcode = if_de_ir[6:0];
    assign de_funct3 = if_de_ir[14:12];
    assign de_funct7_30 = if_de_ir[30];
    assign opcode = de_opcode;  // Connect to existing wire
    
    // Decode instruction struct
    assign de_inst.opcode = opcode_t'(de_opcode);
    assign de_inst.rs1_addr = if_de_ir[19:15];
    assign de_inst.rs2_addr = if_de_ir[24:20];
    assign de_inst.rd_addr = if_de_ir[11:7];
    assign de_inst.funct3 = de_funct3;
    assign de_inst.pc = if_de_pc;
    
    // Determine register usage
    assign de_inst.rs1_used = (de_inst.rs1_addr != 5'b0) && 
                             (de_inst.opcode != LUI) && 
                             (de_inst.opcode != AUIPC) && 
                             (de_inst.opcode != JAL);
                             
    assign de_inst.rs2_used = (de_inst.rs2_addr != 5'b0) && 
                             ((de_inst.opcode == OP) || 
                              (de_inst.opcode == BRANCH) || 
                              (de_inst.opcode == STORE));
                              
    assign de_inst.rd_used = (de_inst.rd_addr != 5'b0) && 
                            (de_inst.opcode != BRANCH) && 
                            (de_inst.opcode != STORE);
    
    // Register file
    logic [31:0] rs1_data, rs2_data, reg_write_data;
    logic [4:0] reg_write_addr;
    logic reg_write_en;
    
    REG_FILE OTTER_REG_FILE (
        .CLK(CLK),
        .EN(reg_write_enable),
        .ADR1(de_inst.rs1_addr),
        .ADR2(de_inst.rs2_addr), 
        .WA(reg_write_addr),
        .WD(reg_write_data),
        .RS1(rs1_data),
        .RS2(rs2_data)
    );
    
    // Immediate generation
    logic [31:0] U_imm, I_imm, S_imm, B_imm, J_imm;
    
    ImmediateGenerator OTTER_IMGEN (
        .IR(if_de_ir[31:7]),
        .U_TYPE(U_imm),
        .I_TYPE(I_imm), 
        .S_TYPE(S_imm),
        .B_TYPE(B_imm),
        .J_TYPE(J_imm)
    );
    
    // Control signal generation
    always_comb begin
        // Default values
        de_inst.alu_fun = 4'b0000;
        de_inst.memWrite = 1'b0;
        de_inst.memRead2 = 1'b0;
        de_inst.regWrite = 1'b0;
        de_inst.rf_wr_sel = 2'b00;
        de_inst.mem_type = 3'b000;
        
        case (de_inst.opcode)
            LUI: begin
                de_inst.alu_fun = 4'b1001;  // Copy operation
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b11;  // ALU result
            end
            AUIPC: begin
                de_inst.alu_fun = 4'b0000;  // Add
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b11;  // ALU result
            end
            JAL: begin
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b00;  // PC + 4
            end
            JALR: begin
                de_inst.alu_fun = 4'b0000;  // Add for address calculation
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b00;  // PC + 4
            end
            LOAD: begin
                de_inst.alu_fun = 4'b0000;  // Add for address
                de_inst.memRead2 = 1'b1;
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b10;  // Memory data
                de_inst.mem_type = {if_de_ir[14], if_de_ir[13:12]}; // sign, size
            end
            STORE: begin
                de_inst.alu_fun = 4'b0000;  // Add for address
                de_inst.memWrite = 1'b1;
                de_inst.mem_type = {1'b0, if_de_ir[13:12]}; // size only
            end
            OP_IMM: begin
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b11;  // ALU result
                case (de_funct3)
                    3'b000: de_inst.alu_fun = 4'b0000;  // ADDI
                    3'b001: de_inst.alu_fun = 4'b0001;  // SLLI
                    3'b010: de_inst.alu_fun = 4'b0010;  // SLTI
                    3'b011: de_inst.alu_fun = 4'b0011;  // SLTIU
                    3'b100: de_inst.alu_fun = 4'b0100;  // XORI
                    3'b101: de_inst.alu_fun = de_funct7_30 ? 4'b1101 : 4'b0101;  // SRAI/SRLI
                    3'b110: de_inst.alu_fun = 4'b0110;  // ORI
                    3'b111: de_inst.alu_fun = 4'b0111;  // ANDI
                endcase
            end
            OP: begin
                de_inst.regWrite = 1'b1;
                de_inst.rf_wr_sel = 2'b11;  // ALU result
                de_inst.alu_fun = {de_funct7_30, de_funct3};
            end
            BRANCH: begin
                // No register write for branches
                de_inst.alu_fun = 4'b1000;  // Subtraction for comparison
            end
        endcase
    end
    
    // ALU operand selection logic
    logic alu_src_a;
    logic [1:0] alu_src_b;
    logic [31:0] alu_op_a_selected, alu_op_b_selected;
    
    // Control signals for ALU operand selection
    always_comb begin
        // Default values
        alu_src_a = 1'b0;  // Default to RS1
        alu_src_b = 2'b00; // Default to RS2
        
        case (de_inst.opcode)
            AUIPC: begin
                alu_src_a = 1'b0;  // Will be overridden below to use PC
                alu_src_b = 2'b11; // U_TYPE immediate  
            end
            LUI: begin
                alu_src_a = 1'b1;  // U_TYPE immediate
                alu_src_b = 2'b01; // Not used, set to immediate
            end
            OP: begin
                alu_src_a = 1'b0;  // RS1
                alu_src_b = 2'b00; // RS2
            end
            STORE: begin
                alu_src_a = 1'b0;  // RS1
                alu_src_b = 2'b10; // S_TYPE immediate
            end
            BRANCH: begin
                alu_src_a = 1'b0;  // RS1
                alu_src_b = 2'b00; // RS2 for comparison
            end
            default: begin // I-type, LOAD, etc.
                alu_src_a = 1'b0;  // RS1
                alu_src_b = 2'b01; // I_TYPE immediate
            end
        endcase
    end
    
    // ALU Operand A selection using TwoMux (special case for AUIPC)
    logic [31:0] two_mux_out;
    TwoMux OTTER_ALU_MUXA (
        .ALU_SRC_A(alu_src_a),
        .RS1(rs1_data),
        .U_TYPE(U_imm),
        .SRC_A(two_mux_out)
    );
    
    // Special handling for AUIPC which needs PC as operand A
    assign alu_op_a_selected = (de_inst.opcode == AUIPC) ? if_de_pc : two_mux_out;
    
    // ALU Operand B selection using FourMux
    FourMux OTTER_ALU_MUXB (
        .SEL(alu_src_b),
        .ZERO(rs2_data),     // RS2
        .ONE(I_imm),         // I_TYPE immediate
        .TWO(S_imm),         // S_TYPE immediate
        .THREE(U_imm),       // U_TYPE immediate
        .OUT(alu_op_b_selected)
    );
    
    // DE/EX Pipeline register
    always_ff @(posedge CLK) begin
        if (RST) begin
            de_ex_inst <= '0;
            de_ex_opA <= 32'b0;
            de_ex_opB <= 32'b0;
            de_ex_rs2 <= 32'b0;
            de_ex_pc <= 32'b0;
            de_ex_i_imm <= 32'b0;
            de_ex_b_imm <= 32'b0;
            de_ex_j_imm <= 32'b0;
        end else begin
            de_ex_inst <= de_inst;
            de_ex_pc <= if_de_pc;
            de_ex_rs2 <= rs2_data;
            de_ex_i_imm <= I_imm;
            de_ex_b_imm <= B_imm;
            de_ex_j_imm <= J_imm;
            de_ex_opA <= alu_op_a_selected;
            de_ex_opB <= alu_op_b_selected;
        end
    end

     
    
	
	
//==== Execute ======================================================
    
    // EX/MEM Pipeline registers
    logic [31:0] ex_mem_rs2, ex_mem_alu_result, ex_mem_pc;
    instr_t ex_mem_inst;
    
    // use pipeline register values directly
    logic [31:0] opA_forwarded, opB_forwarded;
    assign opA_forwarded = de_ex_opA;
    assign opB_forwarded = de_ex_opB; 
    
    // ALU 
    ALU OTTER_ALU (
        .SRC_A(opA_forwarded),
        .SRC_B(opB_forwarded), 
        .ALU_FUN(de_ex_inst.alu_fun),
        .RESULT(aluResult)
    );
    
    // Branch condition generation
    BCG OTTER_BCG (
        .RS1(opA_forwarded),
        .RS2(de_ex_rs2),  // Use rs2 for branch comparison
        .BR_EQ(br_eq),
        .BR_LT(br_lt),
        .BR_LTU(br_ltu)
    );
    
    // Branch/Jump target calculation 
    BAG OTTER_BAG (
        .RS1(opA_forwarded),
        .I_TYPE(de_ex_i_imm),
        .J_TYPE(de_ex_j_imm),
        .B_TYPE(de_ex_b_imm),
        .FROM_PC(de_ex_pc),
        .JAL(jal_target),
        .JALR(jalr_target),
        .BRANCH(branch_target)
    );
    
    // Branch decision logic
    logic branch_taken;
    always_comb begin
        branch_taken = 1'b0;
        if (de_ex_inst.opcode == BRANCH) begin
            case (de_ex_inst.funct3)
                3'b000: branch_taken = br_eq;   // BEQ
                3'b001: branch_taken = ~br_eq;  // BNE  
                3'b100: branch_taken = br_lt;   // BLT
                3'b101: branch_taken = ~br_lt;  // BGE
                3'b110: branch_taken = br_ltu;  // BLTU
                3'b111: branch_taken = ~br_ltu; // BGEU
                default: branch_taken = 1'b0;
            endcase
        end
    end
    
    // PC source selection (this controls PC in IF stage)
    always_comb begin
        pc_source = 3'b000;  // Default PC+4
        
        case (de_ex_inst.opcode)
            JAL: pc_source = 3'b011;                           // JAL target
            JALR: pc_source = 3'b001;                          // JALR target
            BRANCH: pc_source = branch_taken ? 3'b010 : 3'b000; // Branch target or PC+4
            default: pc_source = 3'b000;                       // PC+4
        endcase
    end
    
    // EX/MEM Pipeline register
    always_ff @(posedge CLK) begin
        if (RST) begin
            ex_mem_inst <= '0;
            ex_mem_alu_result <= 32'b0;
            ex_mem_rs2 <= 32'b0;
            ex_mem_pc <= 32'b0;
        end else begin
            ex_mem_inst <= de_ex_inst;
            ex_mem_alu_result <= aluResult;
            ex_mem_rs2 <= de_ex_rs2;
            ex_mem_pc <= de_ex_pc;
        end
    end
     



//==== Memory ======================================================
    
    // MEM/WB Pipeline registers
    logic [31:0] mem_wb_alu_result, mem_wb_mem_data, mem_wb_pc;
    instr_t mem_wb_inst;
    
    // Memory interface
    assign IOBUS_ADDR = ex_mem_alu_result;
    assign IOBUS_OUT = ex_mem_rs2;
    assign mem_write_enable = ex_mem_inst.memWrite;
    assign mem_read2_enable = ex_mem_inst.memRead2;
    assign mem_size = ex_mem_inst.mem_type[1:0];  // Memory access size
    
    // MEM/WB Pipeline register
    always_ff @(posedge CLK) begin
        if (RST) begin
            mem_wb_inst <= '0;
            mem_wb_alu_result <= 32'b0;
            mem_wb_mem_data <= 32'b0;
            mem_wb_pc <= 32'b0;
        end else begin
            mem_wb_inst <= ex_mem_inst;
            mem_wb_alu_result <= ex_mem_alu_result;
            mem_wb_mem_data <= mem_data;
            mem_wb_pc <= ex_mem_pc;
        end
    end
    
 
 
 
     
//==== Write Back ==================================================
    
    // Register write-back connections
    assign reg_write_enable = mem_wb_inst.regWrite;
    assign reg_write_addr = mem_wb_inst.rd_addr;
    
    // Write-back data selection using FourMux
    FourMux OTTER_REG_MUX (
        .SEL(mem_wb_inst.rf_wr_sel),
        .ZERO(mem_wb_pc + 4),           // PC + 4 (for JAL/JALR)
        .ONE(32'b0),                    // Unused 
        .TWO(mem_wb_mem_data),          // Memory data (LOAD)
        .THREE(mem_wb_alu_result),      // ALU result
        .OUT(reg_write_data)
    );
     


 
 

       
            
endmodule
