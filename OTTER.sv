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

module OTTER(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    
    // Init Signals
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,I_immed,S_immed,U_immed,aluBin,aluAin,                aluResult,rfIn,csr_reg, mem_data,IR,mem_addr2, mem_dout2,pc_mux_out,ex_result;;
    wire [31:0] rs1, rs2, utype, itype, stype, btype, de_used_rs1, de_used_rs2;
    wire memRead1,memRead2,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead,opA_sel,mem_RDEN2, mem_WE2,de_jump,         de_branch, de_regWrite, de_memWE2, de_memRDEN2,ex_jump,sign;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize, de_alu_srca, de_rf_wr_sel, alu_srca_sel, size;
    wire [2:0] de_alu_srcb, alu_srcb_sel;
    wire [3:0] alu_fun, de_alu_fun;
    
    logic [31:0] bag_branch, bag_jal, bag_jalr, forwarded_A, forwarded_B, ex_mem_result, ex_forwarded_rs2, jalr_forwarded,                 if_de_pc, if_de_pc_plus4, if_de_ir, rf_write_rd_addr, mem_to_reg_mux_out;
    logic [31:0] de_ex_rs1, de_ex_pc, de_ex_pc_plus4, jtype, de_ex_opA, de_ex_opB, de_ex_rs2, de_ex_utype, de_ex_itype,                 de_ex_stype, de_ex_btype, de_ex_jtype, de_ex_alu_b;
    logic [31:0] opA_forwarded, opB_forwarded, ex_mem_pc_plus4, ex_mem_rs2, mem_wb_dout2, mem_wb_pc_plus4, mem_wb_result;
    logic [6:0] de_ex_opcode;
    logic [4:0] mem_wb_rd_addr, de_ex_rs1_addr, de_ex_rs2_addr, de_ex_rd_addr, ex_mem_rd_addr;
    logic [3:0] de_ex_alu_fun;
    logic [2:0] de_ex_func3;
    logic [1:0] pc_sel, de_ex_rf_wr_sel, de_ex_size, forward_A_SEL, forward_B_SEL, ex_mem_rf_wr_sel, ex_mem_size,                mem_wb_rf_wr_sel, jalr_sel, forward_rs2;
    logic pcWrite,ex_mem_memRDEN2, ex_mem_regWrite, ex_mem_memWE2,br_eq, br_lt, br_ltu,de_ex_flushed,flush_de_ex,          flush_if_de,stall,if_de_flushed, flush_next_if_de;
    logic mem_wb_reg_write,de_ex_jump, de_ex_branch, de_ex_regWrite, de_ex_memWE2, de_ex_memRDEN2, de_store, de_ex_store,          imm_A, imm_B,de_ex_sign, jal_taken, jalr_taken, branch_taken, de_load, de_ex_load, ex_mem_load;
    logic ex_mem_sign, mem_wb_regWrite, de_ex_stalled;
    logic ex_mem_aluRes = 0;
   
//==== Instruction Fetch ===========================================

     assign if_de_ir = IR;
     assign pcWrite = ~stall; 	
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
     always_ff @(posedge CLK) begin
                if (!stall) begin
                    if_de_pc <= pc;
                    if_de_pc_plus4 <= pc+4;
                end
                if (flush_next_if_de || flush_de_ex) if_de_flushed = 1;
                else if_de_flushed = 0;
     end
     
     always_ff @(posedge CLK) begin
        if (flush_if_de) flush_next_if_de = 1;
        else flush_next_if_de = 0;
     end
     
     
//==== Instruction Decode ===========================================

    assign utype = {if_de_ir[31:12],12'b0};
    assign itype = {{21{if_de_ir[31]}}, if_de_ir[30:20]};
    assign stype = {{21{if_de_ir[31]}},if_de_ir[30:25],if_de_ir[11:7]};
    assign btype = {{20{if_de_ir[31]}},if_de_ir[7],if_de_ir[30:25],if_de_ir[11:8],1'b0};
    assign jtype = {{12{if_de_ir[31]}},if_de_ir[19:12],if_de_ir[20],if_de_ir[30:21],1'b0};
    
    CU_DCDR Decoder (
                    .ir_opcode_dcdr(if_de_ir[6:0]),
                    .ir_funct_dcdr(if_de_ir[14:12]),
                    .ir_30_dcdr(if_de_ir[30]),
                    .int_taken_dcdr(1'b0),
                    .jump(de_jump),
                    .branch(de_branch),
                    .store(de_store),
                    .regWrite(de_regWrite),
                    .memWE2(de_memWE2),
                    .memRDEN2(de_memRDEN2),
                    .alu_fun_dcdr(de_alu_fun),
                    .alu_scra_dcdr(alu_srca_sel),
                    .alu_scrb_dcdr(alu_srcb_sel),
                    .rf_wr_sel_dcdr(de_rf_wr_sel)
                    );
    
    
    RegFile RegFile (
                    .w_en(mem_wb_reg_write),
                    .adr1(if_de_ir[19:15]),
                    .adr2(if_de_ir[24:20]),
                    .w_adr(mem_wb_rd_addr),
                    .w_data(mem_to_reg_mux_out),
                    .clk(CLK),
                    .rs1(rs1),
                    .rs2(rs2)
                    );
    
    TwoMux DE_ALU_A_MUX (
                      .RS1(rs1),
                      .U_TYPE(utype),
                      .ALU_SRC_A(alu_srca_sel[0]),
                      .SRC_A(de_used_rs1)
                      );
                      
    FourMux DE_ALU_B_MUX (
                      .ZERO(rs2),
                      .ONE(itype),
                      .TWO(stype),
                      .THREE(if_de_pc),
                      .SEL(alu_srcb_sel[1:0]),
                      .OUT(de_used_rs2)
                      );
    
    // Branch conditional generator
    always_comb begin
          de_load = 0;
          branch_taken = 0;
          
          if (if_de_ir[6:0] == 7'b0000011) de_load = 1;
   
          if (de_ex_branch && !de_ex_flushed) begin
          // Not jump instruction (branch instead)
            begin
              case (de_ex_func3)
              // BEQ
              3'b000: begin
                 branch_taken = forwarded_A == forwarded_B ? 1 : 0;
              end
              // BNE
              3'b001: begin
                branch_taken = forwarded_A == forwarded_B ? 0 : 1;
              end
              // BLT
              3'b100: begin
                branch_taken = $signed(forwarded_A) < $signed(forwarded_B) ? 1 : 0;
              end
              // BGE
              3'b101: begin
                branch_taken = $signed(forwarded_A) < $signed(forwarded_B) ? 0 : 1;
              end
              // BLTU
              3'b110: begin
                branch_taken = forwarded_A < forwarded_B ? 1 : 0;
              end
              // BGEU
              3'b111: begin
                branch_taken = forwarded_A < forwarded_B ? 0 : 1;
              end
              default: begin
                branch_taken = 0;
              end
              endcase
          end
          end
    end
    
    // Jump conditional
    always_comb begin
    jal_taken = 0;
    jalr_taken = 0;
    if (de_jump == 1 && !flush_next_if_de) begin
             if (if_de_ir[6:0] == 7'b1101111) begin
             // JAL opcode
                jal_taken = 1;
             end else begin
             // JALR
                jalr_taken = 1;
             end
         end
    end
    
    HazardUnit HazardUnit(
               .rs1_in(de_ex_rs1_addr),
               .rs2_in(de_ex_rs2_addr),
               .de_rd(de_ex_rd_addr),
               .de_rd_reg_write(de_ex_regWrite),
               .ex_rd(ex_mem_rd_addr),
               .ex_rd_reg_write(ex_mem_regWrite),
               .mem_rd(mem_wb_rd_addr),
               .mem_rd_reg_write(mem_wb_reg_write),
               .load(ex_mem_load),
               .jal_taken(jal_taken),
               .jalr_taken(jalr_taken),
               .branch(branch_taken),
               .store(de_ex_store),
               .forward_rs2(forward_rs2),
               .de_ex_stalled(de_ex_stalled),
               .de_ex_load(de_ex_load),
               .imm_A(imm_A),
               .imm_B(imm_B),
               .pc_mux_out(pc_sel),
               .de_rs1_jalr(if_de_ir[19:15]),
               .jalr_sel(jalr_sel),
               .flush_if_de(flush_if_de),
               .flush_de_ex(flush_de_ex),
               .stall(stall),
               .alu_sel_1(forward_A_SEL),
               .alu_sel_2(forward_B_SEL)
    );
    
    
    always_ff @(posedge CLK) begin
        if (!stall) begin
            de_ex_stalled <= 0;
            // Assign used values
            de_ex_rs1_addr <= if_de_ir[19:15];
            de_ex_rs1 <= de_used_rs1;
            de_ex_alu_b <= de_used_rs2;
            de_ex_rs2 <= rs2;
            de_ex_rs2_addr <= if_de_ir[24:20];
            de_ex_pc <= if_de_pc;
            de_ex_pc_plus4 <= if_de_pc_plus4;
            de_ex_opcode <= if_de_ir[6:0];
            de_ex_func3 <= if_de_ir[14:12];
            de_ex_sign <= if_de_ir[14];
            de_ex_size <= if_de_ir[13:12];
            de_ex_load <= de_load;
            de_ex_store <= de_store;
            if (alu_srca_sel != 0) imm_A <= 1;
            else imm_A <= 0;
            
            if (alu_srcb_sel != 0) imm_B <= 1;
            else imm_B <= 0;
            
            // Assign control values
            de_ex_branch <= de_branch;
            if (if_de_flushed) begin
                de_ex_regWrite <= 0;
                de_ex_memWE2 <= 0;
                de_ex_flushed <= 1;
            end
            else begin
                de_ex_regWrite <= de_regWrite;
                de_ex_memWE2 <= de_memWE2;
                de_ex_flushed <= 0;
            end
            de_ex_memRDEN2 <= de_memRDEN2;
            de_ex_alu_fun <= de_alu_fun;
            de_ex_rf_wr_sel <= de_rf_wr_sel;
            
            // Assign immediate values in DE_EX buffer
            de_ex_itype <= itype;
            de_ex_btype <= btype;
            de_ex_jtype <= jtype;
            
            // Assign register addresses
            de_ex_rd_addr <= if_de_ir[11:7];
            
            
        end else begin
            de_ex_stalled <= 1;
            end
        
    end
     
    
	
	
//==== Execute ======================================================

    assign ex_jump = de_ex_jump;
     
    BRANCH_ADDR_GEN BAG (
                   .J_TYPE_IMM(jtype),
                   .B_TYPE_IMM(de_ex_btype),
                   .I_TYPE_IMM(itype), // Immediately decoded
                   .rs1(jalr_forwarded),
                   .PC_B(de_ex_pc),
                   .PC_J(if_de_pc),
                   .branch(bag_branch),
                   .jal(bag_jal),
                   .jalr(bag_jalr)
                   );
                   
    FourMux JALR_forward (
                    .ZERO(rs1),
                    .ONE(ex_result),
                    .TWO(ex_mem_result),
                    .THREE(mem_to_reg_mux_out),
                    .SEL(jalr_sel),
                    .OUT(jalr_forwarded)
                    );

    FourMux PC_MUX (.ZERO (pc+4),
                    .ONE (bag_jalr),
                    .TWO (bag_branch),
                    .THREE (bag_jal),
                    .SEL (pc_sel),
                    .OUT (pc_mux_out)
                    );
                    
    FourMux ALU_forward_A (
                    .ZERO (de_ex_rs1),
                    .ONE (ex_mem_result),
                    .TWO (mem_to_reg_mux_out),
                    .THREE (0),
                    .SEL (forward_A_SEL),
                    .OUT (forwarded_A)
                    );
    
    FourMux ALU_forward_B (
                    .ZERO (de_ex_alu_b),
                    .ONE (ex_mem_result),
                    .TWO (mem_to_reg_mux_out),
                    .THREE (0),
                    .SEL (forward_B_SEL),
                    .OUT (forwarded_B)
                    );
                    
    FourMux MEM_forward_rs2 (
                    .ZERO (de_ex_rs2),
                    .ONE (ex_mem_result),
                    .TWO (mem_to_reg_mux_out),
                    .THREE (0),
                    .SEL (forward_rs2),
                    .OUT (ex_forwarded_rs2)
                    );

    PC PC_mod (
           .PC_RST(RESET),
           .PC_WE(pcWrite), 
           .PC_DIN(pc_mux_out),
           .CLK(CLK), 
           .PC_COUNT(pc)
           );
    
    ALU ALU (
            .srcA(forwarded_A),
            .srcB(forwarded_B),
            .alu_fun(de_ex_alu_fun),
            .alu_result(ex_result)
        );
        
    always_ff @(posedge CLK) begin
        ex_mem_result <= ex_result;
        ex_mem_rs2 <= ex_forwarded_rs2;
        ex_mem_load <= de_ex_load;
        
        ex_mem_memRDEN2 <= de_ex_memRDEN2;
        ex_mem_regWrite <= de_ex_regWrite;
        ex_mem_rf_wr_sel <= de_ex_rf_wr_sel;
        ex_mem_memWE2 <= de_ex_memWE2;
        
        ex_mem_size <= de_ex_size;
        ex_mem_sign <= de_ex_sign;
        
        ex_mem_pc_plus4 <= de_ex_pc;
        ex_mem_rd_addr <= de_ex_rd_addr;
        
    end



//==== Memory ======================================================
     
    assign mem_RDEN2 = ex_mem_memRDEN2;
    assign mem_WE2 = ex_mem_memWE2;
    assign mem_addr2 = ex_mem_result;
    assign size = ex_mem_size;
    assign sign = ex_mem_sign;
    assign IOBUS_ADDR = ex_mem_result;
    assign IOBUS_OUT = ex_mem_rs2;
    assign mem_wb_dout2 = mem_dout2;
    
    
    OTTER_mem_byte OTTER_MEMORY(
                .MEM_CLK(CLK),
                .MEM_ADDR1(pc),
                .MEM_ADDR2(mem_addr2),
                .MEM_DIN2(ex_mem_rs2),
                .MEM_WRITE2(mem_WE2),
                .MEM_READ1(memRead1 && !stall),
                .MEM_READ2(mem_RDEN2),
                .MEM_DOUT1(IR),
                .MEM_DOUT2(mem_dout2),
                .IO_IN(IOBUS_IN),
                .IO_WR(IOBUS_WR),
                .MEM_SIZE(size),
                .MEM_SIGN(sign)
                );
    
    always_ff @(posedge CLK) begin
        mem_wb_rd_addr <= ex_mem_rd_addr;
        mem_wb_pc_plus4 <= ex_mem_pc_plus4;
        mem_wb_result <= ex_mem_result;
        
        mem_wb_rf_wr_sel <= ex_mem_rf_wr_sel;
        mem_wb_regWrite <= ex_mem_regWrite;
        
    end
 
     
//==== Write Back ==================================================

    assign rf_write_rd_addr = mem_wb_rd_addr;
    assign mem_wb_reg_write = mem_wb_regWrite;
    
    FourMux RF_WRITE_MUX (
               .ZERO(ex_mem_pc_plus4),
               .ONE(0),
               .TWO(mem_wb_dout2),
               .THREE(mem_wb_result),
               .SEL(mem_wb_rf_wr_sel),
               .OUT(mem_to_reg_mux_out)
               );
            
endmodule