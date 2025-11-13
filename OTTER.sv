`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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
    logic [31:0] pc;
} instr_t;

module OTTER(input CLK,
    input RST,
    input [31:0] IOBUS_IN,
    output [31:0] IOBUS_OUT,
    output [31:0] IOBUS_ADDR,
    output logic IOBUS_WR 
);          
 
    logic [6:0] opcode;
    
    logic [31:0] pc, 
                 pc_value, 
                 next_pc, 
                 jalr_pc, 
                 branch_pc, 
                 jump_pc, 
                 int_pc,
                 A,
                 B,
                 I_immed,
                 S_immed,
                 U_immed,
                 aluBin,
                 aluAin,
                 aluResult,
                 rfIn,
                 csr_reg, 
                 mem_data;
    
    logic [31:0] IR;
    
    logic memRead1,memRead2;
    
    logic pcWrite, 
          regWrite, 
          memWrite, 
          op1_sel, 
          mem_op, 
          IorD, 
          pcWriteCond, 
          memRead;
    
    logic [1:0] opB_sel, rf_sel, wb_sel, mSize;
    
    logic [2:0] pc_sel;
    
    logic [3:0]alu_fun;
    
    logic opA_sel;
    
    logic br_taken, br_lt, br_eq, br_ltu;
              
//==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc;
     
     assign pcWrite = 1'b1; 	//Hardwired high, assuming no hazards
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
    FourMux PC_MUX (
        .ZERO(pc_value + 4),
        .ONE(jalr_pc),
        .TWO(branch_pc),
        .THREE(jump_pc),
        .SEL(pc_sel),
        .OUT(pc)
    );

    // this needs to instantiate our PC.sv module    
    PC OTTER_PC (
        .CLK(CLK), 
        .RST(RST), 
        .PC_LD(pcWrite), 
        .PC_IN(pc), 
        .PC_OUT(pc_value)
    );
   

    // PC register
    always_ff @(posedge CLK) begin
        if_de_pc <= pc_value;
    end

//==== Instruction Decode ===========================================

    logic regWrite_D;
    
    instr_t de_ex_inst, de_inst;

    opcode_t decoded_opcode;
    
    always_comb begin
        decoded_opcode = opcode_t'(IR[6:0]);
    end
    
    assign de_inst.o
    pcode = decoded_opcode;

    REG_FILE OTTER_REG_FILE(
        .CLK(CLK), 
        .EN(regWrite), 
        .ADR1(IR[19:15]), 
        .ADR2(IR[24:20]), 
        .WA(mem_wb_inst.rd_addr), 
        .WD(rfIn), 
        .RS1(A), 
        .RS2(B)
    );



    // init control Units
    ControlHazardUnit CHU(
        .CLK(CLK),
        .de_opcode(de_inst.opcode),
        .ex_func3(),
        .BR_EQ(br_eq),
        .BR_LT(br_lt),
        .BR_LTU(br_ltu),
        .ex_I_Immed(),
        .ex_rs1(),
        .ex_pc(),
        .ex_B_Immed(),
        .branch_flag(),
        .jalr_pc(),
        .branch_pc(),
        .jalr_pc(),
        .pc_sel(),
        .flush(),
        .deflush()
    );


    DataHazardUnit DHU (
        .opcode(),
        .de_adr1(),
        .de_adr2(),
        .ex_adr1(),
        .ex_adr2(),
        .ex_rd(),
        .mem_rd(),
        .wb_rd(),
        .ex_regWrite(),
        .mem_regWrite(),
        .wb_regWrite(),
        .de_rs1_used(),
        .de_rs2_used(),
        .ex_rs1_used(),
        .ex_rs2_used(),
        .ex_mem_IR(),
        .de_ex__IR(),
        .de_ex_aluRes(),
        .fsel1(),
        .fsel2(),
        .load_use_haz()
    );



    CU_DCDR decoder_unit (
        .IR_30(IR[30]),
        .IR_OPCODE(IR[6:0]),
        .IR_FUNCT(IR[14:12]),
        .BR_EQ(br_eq),
        .BR_LT(br_lt),
        .BR_LTU(br_ltu),
        .ALU_FUN(alu_fun),
        .ALU_SRCA(opA_sel),
        .ALU_SRCB(opB_sel),
        .PC_SOURCE(pc_sel),
        .RF_WR_SEL(wb_sel),
        .REG_WRITE(regWrite_D),
        .MEM_WRITE(memWrite),
        .MEM_READ2(memRead2)
    );
            
    assign I_immed = {{20{IR[31]}}, IR[31:20]};
    assign S_immed = {{20{IR[31]}}, IR[31:25], IR[11:7]};
    assign U_immed = {IR[31:12], 12'b0};

    assign de_inst.rs1_addr    = IR[19:15];
    assign de_inst.rs2_addr    = IR[24:20];
    assign de_inst.rd_addr     = IR[11:7];
    assign de_inst.rs1_used    = (IR[19:15] != 0) &&
                                 (decoded_opcode  != LUI) &&
                                 (decoded_opcode   != AUIPC) &&
                                 (decoded_opcode   != JAL);
    assign de_inst.rs2_used    = (IR[24:20] != 0) ||
                                 (decoded_opcode   == BRANCH) ||
                                 (decoded_opcode   == STORE) ||
                                 (decoded_opcode   == OP);
    assign de_inst.rd_used     = (IR[11:7] != 0) &&
                                 (decoded_opcode   != STORE) &&
                                 (decoded_opcode   != BRANCH);
    assign de_inst.alu_fun     = alu_fun;
    assign de_inst.memWrite    = memWrite;
    assign de_inst.memRead2    = memRead2;
    assign de_inst.regWrite    = regWrite_D;
    assign de_inst.rf_wr_sel   = wb_sel;
    assign de_inst.mem_type    = IR[14:12];  // funct3
    assign de_inst.pc          = if_de_pc;


    // this should be TWO_MUX.sv
    TwoMux SRCA_MUX (
        .SRCA_MUX0(A),
        .SRCA_MUX1(U_immed),
        .Alu_srcASEL(opA_sel),
        .SRCA_OUT(aluAin)
        );
        
     // this should be FOUR_MUX.sv   
     FourMux SRCB_MUX(
        .ZERO(B),
        .ONE(I_immed),
        .TWO(S_immed),
        .THREE(if_de_pc),
        .SEL(opB_sel),
        .OUT(aluBin)
        );

 
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_IR;
    logic [31:0] de_ex_rs2;
    logic [31:0] de_ex_I_immed;
    logic [31:0] de_ex_pc; 
    

    always_ff @ (posedge CLK) begin
        de_ex_inst <= de_inst;
        de_ex_opA <= aluAin;
        de_ex_opB <= aluBin;
        de_ex_IR <= IR;
        de_ex_rs2 <= B;
        de_ex_I_immed <= I_immed;
        de_ex_pc <= if_de_pc;
    end
    
	
//==== Execute ======================================================

     logic [31:0] ex_mem_rs2;
     logic [2:0] func_3;
     logic [31:0] ex_mem_aluRes;
     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     
     // no forwarding in lab 2
     assign opA_forwarded = de_ex_opA;
     assign opB_forwarded = de_ex_opB;
     
     assign jalr_pc = de_ex_I_immed + de_ex_opA;
     assign branch_pc = de_ex_inst.pc + {{20{de_ex_IR[31]}}, de_ex_IR[7], de_ex_IR[30:25], de_ex_IR[11:8], 1'b0};
     assign jump_pc = de_ex_inst.pc + {{12{de_ex_IR[31]}}, de_ex_IR[19:12], de_ex_IR[20], de_ex_IR[30:21], 1'b0};
     assign int_pc = 0; 
     
    // Branch comparison logic
    always_comb begin
         br_eq  = (opA_forwarded == opB_forwarded);
         br_lt  = ($signed(opA_forwarded) < $signed(opB_forwarded));
         br_ltu = (opA_forwarded < opB_forwarded);

    end

    assign func_3 = de_ex_IR[14:12];
     
     // Creates a RISC-V ALU
    ALU ALU (
        .SRC_A(opA_forwarded),
        .SRC_B(opB_forwarded),
        .ALU_FUN(de_ex_inst.alu_fun),
        .RESULT(aluResult)
    );
     
    always_ff @(posedge CLK) begin
        ex_mem_inst.opcode     <= de_ex_inst.opcode;
        ex_mem_inst.alu_fun    <= de_ex_inst.alu_fun;
        ex_mem_inst.rs2_addr   <= de_ex_inst.rs2_addr;
        ex_mem_inst.rd_addr    <= de_ex_inst.rd_addr;
        ex_mem_inst.rf_wr_sel  <= de_ex_inst.rf_wr_sel;
        ex_mem_inst.mem_type   <= de_ex_inst.mem_type;
        ex_mem_inst.memWrite   <= de_ex_inst.memWrite;
        ex_mem_inst.memRead2   <= de_ex_inst.memRead2;
        ex_mem_inst.regWrite   <= de_ex_inst.regWrite;
        ex_mem_inst.pc         <= de_ex_inst.pc;
    
        ex_mem_aluRes <= aluResult;
        ex_mem_rs2    <= de_ex_rs2;
    end


//==== Memory ======================================================
     
    instr_t mem_wb_inst;
         
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    assign IOBUS_WR = ex_mem_inst.memWrite;
    
    logic [31:0] WB_aluResult, WB_memData, WB_pc_plus_4;

    
    Memory OTTER_MEMORY (
        .MEM_CLK   (CLK),
        .MEM_RDEN1 (memRead1),
        .MEM_ADDR1 (pc_value[15:2]),       
        .MEM_DOUT1 (IR),            
        .MEM_RDEN2 (ex_mem_inst.memRead2),
        .MEM_WE2   (ex_mem_inst.memWrite),
        .MEM_ADDR2 (ex_mem_aluRes),
        .MEM_DIN2  (ex_mem_rs2),
        .MEM_SIZE  (ex_mem_inst.mem_type[1:0]),
        .MEM_SIGN  (ex_mem_inst.mem_type[2]),   
        .IO_IN     (IOBUS_IN),
        .IO_WR     (IOBUS_WR),
        .MEM_DOUT2 (mem_data)
    );
    
    always_ff @(posedge CLK) begin
        mem_wb_inst.rd_addr   <= ex_mem_inst.rd_addr;
        mem_wb_inst.rf_wr_sel <= ex_mem_inst.rf_wr_sel;
        mem_wb_inst.mem_type  <= ex_mem_inst.mem_type;
        mem_wb_inst.memWrite  <= ex_mem_inst.memWrite;
        mem_wb_inst.memRead2  <= ex_mem_inst.memRead2;
        mem_wb_inst.regWrite  <= ex_mem_inst.regWrite;
        mem_wb_inst.pc        <= ex_mem_inst.pc;
    
        WB_aluResult <= ex_mem_aluRes;
        WB_memData   <= mem_data;
        WB_pc_plus_4 <= ex_mem_inst.pc + 4;
    end



     
//==== Write Back ==================================================
     
     logic [31:0] WB_rfIn;
     logic [31:0] MEM_PC;
     
     assign MEM_PC = mem_wb_inst.pc + 4;
     
     FourMux RegMux (.SEL(mem_wb_inst.rf_wr_sel),
                     .ZERO(MEM_PC),
                     .ONE(32'b0),
                     .TWO(mem_data),
                     .THREE(WB_aluResult),
                     .OUT(WB_rfIn)
                     );
                     
     
    assign rfIn = WB_rfIn;               
    assign regWrite = mem_wb_inst.regWrite; 
    assign rf_sel = mem_wb_inst.rd_addr;    
     
endmodule
