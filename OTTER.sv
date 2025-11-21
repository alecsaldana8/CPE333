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
    
    // Hazard detection and control signals (declared early for use throughout pipeline)
    logic [1:0] fsel1, fsel2;  // Forwarding select signals from DataHazardUnit
    logic load_use_haz;        // Load-use hazard signal
    logic [1:0] pc_sel;        // PC select from ControlHazardUnit  
    logic flush, de_flush;     // Flush signals
    logic stall_pipeline;      // Pipeline stall signal
    
    // Forward declarations for pipeline registers used in forwarding
    logic [31:0] mem_wb_pc;
    
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
    
    // IF/DE Pipeline Register (with stall and flush support)
    always_ff @(posedge CLK) begin
        if (RST) begin
            if_de_pc <= 32'b0;
            if_de_ir <= 32'b0;
        end else if (flush) begin
            // Flush pipeline on control hazard
            if_de_pc <= 32'b0;
            if_de_ir <= 32'h00000013;  // NOP instruction
        end else if (!stall_pipeline) begin
            // Normal operation (no stall)
            if_de_pc <= pc;
            if_de_ir <= IR;  // IR comes from memory
        end
        // If stall_pipeline is high, keep current values (stall)
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
    
    // Control Unit Decoder instantiation
    logic [2:0] cu_pc_source;  // PC_SOURCE from decoder
    logic alu_src_a;           // ALU source A select from CU_DCDR
    logic [1:0] alu_src_b;     // ALU source B select from CU_DCDR
    
    CU_DCDR OTTER_CU_DCDR (
        .IR_30(de_funct7_30),
        .IR_OPCODE(de_opcode),
        .IR_FUNCT(de_funct3),
        .BR_EQ(br_eq),
        .BR_LT(br_lt),
        .BR_LTU(br_ltu),
        .ALU_FUN(de_inst.alu_fun),
        .ALU_SRCA(alu_src_a),
        .ALU_SRCB(alu_src_b),
        .PC_SOURCE(cu_pc_source),
        .RF_WR_SEL(de_inst.rf_wr_sel),
        .REG_WRITE(de_inst.regWrite),
        .MEM_WRITE(de_inst.memWrite),
        .MEM_READ2(de_inst.memRead2)
    );
    
    // Memory type generation (not handled by CU_DCDR)
    always_comb begin
        case (de_inst.opcode)
            LOAD: begin
                de_inst.mem_type = {if_de_ir[14], if_de_ir[13:12]}; // sign, size
            end
            STORE: begin
                de_inst.mem_type = {1'b0, if_de_ir[13:12]}; // size only
            end
            default: begin
                de_inst.mem_type = 3'b000;
            end
        endcase
    end
    
    // ALU operand selection signals (now generated by CU_DCDR)
    logic [31:0] alu_op_a_selected, alu_op_b_selected;
    
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
    
    // DE/EX Pipeline register (with stall and flush support)
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
        end else if (de_flush) begin
            // Flush DE/EX stage on control hazard
            de_ex_inst <= '0;
            de_ex_opA <= 32'b0;
            de_ex_opB <= 32'b0;
            de_ex_rs2 <= 32'b0;
            de_ex_pc <= 32'b0;
            de_ex_i_imm <= 32'b0;
            de_ex_b_imm <= 32'b0;
            de_ex_j_imm <= 32'b0;
        end else if (!stall_pipeline) begin
            // Normal operation (no stall)
            de_ex_inst <= de_inst;
            de_ex_pc <= if_de_pc;
            de_ex_rs2 <= rs2_data;
            de_ex_i_imm <= I_imm;
            de_ex_b_imm <= B_imm;
            de_ex_j_imm <= J_imm;
            de_ex_opA <= alu_op_a_selected;
            de_ex_opB <= alu_op_b_selected;
        end
        // If stall_pipeline is high, keep current values (stall)
    end

     
    
	
	
//==== Execute ======================================================
    
    // EX/MEM Pipeline registers
    logic [31:0] ex_mem_rs2, ex_mem_alu_result, ex_mem_pc;
    instr_t ex_mem_inst;
    
    // Forwarded operands
    logic [31:0] opA_forwarded, opB_forwarded;
    
    // Data Hazard Unit instantiation
    DataHazardUnit data_hazard_unit (
        .opcode({de_ex_inst.opcode}),
        .de_adr1(de_ex_inst.rs1_addr),
        .de_adr2(de_ex_inst.rs2_addr),
        .ex_adr1(de_ex_inst.rs1_addr),
        .ex_adr2(de_ex_inst.rs2_addr),
        .ex_rd(de_ex_inst.rd_addr),
        .mem_rd(ex_mem_inst.rd_addr),
        .wb_rd(mem_wb_inst.rd_addr),
        .ex_regWrite(de_ex_inst.regWrite),
        .mem_regWrite(ex_mem_inst.regWrite),
        .wb_regWrite(mem_wb_inst.regWrite),
        .de_rs1_used(de_ex_inst.rs1_used),
        .de_rs2_used(de_ex_inst.rs2_used),
        .ex_rs1_used(de_ex_inst.rs1_used),
        .ex_rs2_used(de_ex_inst.rs2_used),
        .ex_mem_IR({{25{1'b0}}, ex_mem_inst.opcode}),
        .de_ex_IR({{25{1'b0}}, de_ex_inst.opcode}),
        .de_ex_aluRes(aluResult),
        .fsel1(fsel1),
        .fsel2(fsel2),
        .load_use_haz(load_use_haz)
    );
    
    // Control Hazard Unit instantiation
    logic [31:0] chu_jalr_target, chu_branch_target, chu_jal_target;
    ControlHazardUnit control_hazard_unit (
        .CLK(CLK),
        .de_opcode({de_inst.opcode}),
        .ex_opcode({de_ex_inst.opcode}),
        .ex_func3(de_ex_inst.funct3),
        .BR_EQ(br_eq),
        .BR_LT(br_lt),
        .BR_LTU(br_ltu),
        .ex_I_Immed(de_ex_i_imm),
        .ex_rs1(opA_forwarded),
        .ex_pc(de_ex_pc),
        .ex_B_Immed(de_ex_b_imm),
        .de_pc(if_de_pc),
        .de_J_Immed(J_imm),
        .branch_flag(1'b0),
        .jalr_pc(chu_jalr_target),
        .branch_pc(chu_branch_target),
        .jal_pc(chu_jal_target),
        .pc_sel(pc_sel),
        .flush(flush),
        .de_flush(de_flush)
    );
    
    // Pipeline stall logic
    assign stall_pipeline = load_use_haz;
    
    // Forwarding multiplexers for operand A
    always_comb begin
        case (fsel1)
            2'b00: opA_forwarded = de_ex_opA;           // No forwarding
            2'b01: opA_forwarded = aluResult;           // Forward from EX stage (current ALU result)
            2'b10: opA_forwarded = ex_mem_alu_result;   // Forward from MEM stage
            2'b11: opA_forwarded = mem_wb_pc + 4;       // Forward PC+4 for JAL
            default: opA_forwarded = de_ex_opA;
        endcase
    end
    
    // Forwarding multiplexers for operand B  
    always_comb begin
        case (fsel2)
            2'b00: opB_forwarded = de_ex_opB;           // No forwarding
            2'b01: opB_forwarded = aluResult;           // Forward from EX stage (current ALU result)
            2'b10: opB_forwarded = ex_mem_alu_result;   // Forward from MEM stage
            2'b11: opB_forwarded = mem_wb_pc + 4;       // Forward PC+4 for JAL
            default: opB_forwarded = de_ex_opB;
        endcase
    end 
    
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
    logic [31:0] bag_jal_target, bag_jalr_target, bag_branch_target;
    BAG OTTER_BAG (
        .RS1(opA_forwarded),
        .I_TYPE(de_ex_i_imm),
        .J_TYPE(de_ex_j_imm),
        .B_TYPE(de_ex_b_imm),
        .FROM_PC(de_ex_pc),
        .JAL(bag_jal_target),
        .JALR(bag_jalr_target),
        .BRANCH(bag_branch_target)
    );
    
    // Select between ControlHazardUnit and BAG outputs based on hazard detection
    assign jal_target = (flush || de_flush) ? chu_jal_target : bag_jal_target;
    assign jalr_target = (flush || de_flush) ? chu_jalr_target : bag_jalr_target;
    assign branch_target = (flush || de_flush) ? chu_branch_target : bag_branch_target;
    
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
    
    // PC source selection (controlled by ControlHazardUnit or CU_DCDR)
    always_comb begin
        if (flush || de_flush) begin
            // Use ControlHazardUnit output when hazard detected
            case (pc_sel)
                2'b00: pc_source = 3'b000;  // PC+4
                2'b01: pc_source = 3'b001;  // JALR target
                2'b10: pc_source = 3'b010;  // Branch target
                2'b11: pc_source = 3'b011;  // JAL target
                default: pc_source = 3'b000; // Default PC+4
            endcase
        end else begin
            // Use CU_DCDR output for normal operation
            pc_source = cu_pc_source;
        end
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
    logic [31:0] mem_wb_alu_result, mem_wb_mem_data;
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