`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: OTTER_CPU
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
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data,IR,mem_addr2, mem_dout2,pc_mux_out,ex_result;;
    wire [31:0] rs1, rs2, utype, itype, stype, btype, de_used_rs1, de_used_rs2;
    wire memRead1,memRead2,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead,opA_sel,mem_RDEN2, mem_WE2,de_jump, de_branch, de_regWrite, de_memWE2, de_memRDEN2,ex_jump,sign;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize, de_alu_srca, de_rf_wr_sel, alu_srca_sel, size;
    wire [2:0] de_alu_srcb, alu_srcb_sel;
    wire [3:0] alu_fun, de_alu_fun;
    
    logic [31:0] bag_branch, bag_jal, bag_jalr, forwarded_A, forwarded_B, ex_mem_result, ex_forwarded_rs2, jalr_forwarded, if_de_pc, if_de_pc_plus4, if_de_ir, rf_write_rd_addr, mem_to_reg_mux_out;
    logic [31:0] de_ex_rs1, de_ex_pc, de_ex_pc_plus4, jtype, de_ex_opA, de_ex_opB, de_ex_rs2, de_ex_utype, de_ex_itype, de_ex_stype, de_ex_btype, de_ex_jtype, de_ex_alu_b;
    logic [31:0] opA_forwarded, opB_forwarded, ex_mem_pc_plus4, ex_mem_rs2, mem_wb_dout2, mem_wb_pc_plus4, mem_wb_result;
    logic [6:0] de_ex_opcode;
    logic [4:0] mem_wb_rd_addr, de_ex_rs1_addr, de_ex_rs2_addr, de_ex_rd_addr, ex_mem_rd_addr;
    logic [3:0] de_ex_alu_fun;
    logic [2:0] de_ex_func3;
    logic [1:0] pc_sel, de_ex_rf_wr_sel, de_ex_size, forward_A_SEL, forward_B_SEL, ex_mem_rf_wr_sel, ex_mem_size, mem_wb_rf_wr_sel, jalr_sel, forward_rs2;
    logic pcWrite,ex_mem_memRDEN2, ex_mem_regWrite, ex_mem_memWE2,br_eq, br_lt, br_ltu,de_ex_flushed,flush_de_ex, flush_if_de,stall,if_de_flushed, flush_next_if_de;
    logic mem_wb_reg_write,de_ex_jump, de_ex_branch, de_ex_regWrite, de_ex_memWE2, de_ex_memRDEN2, de_store, de_ex_store,imm_A, imm_B,de_ex_sign, jal_taken, jalr_taken, branch_taken, de_load, de_ex_load, ex_mem_load;
    logic ex_mem_sign, mem_wb_regWrite, de_ex_stalled, ex_mem_stalled, mem_out_stalled;
    logic ex_mem_aluRes = 0;
    logic data_cache_stall, cache_stall, if_flushed;
   
//===================== Instruction Fetch =====================

     assign pcWrite = ~(stall || cache_stall || data_cache_stall); 	
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     always_ff @(posedge CLK) begin
            if (flush_if_de) if_flushed = 1;
            else if_flushed = 0;
     end
     
     always_ff @(posedge CLK) begin
                
                if (!(stall || cache_stall || data_cache_stall)) begin
                    if_de_flushed <= if_flushed;
                    if_de_ir <= IR;
                    if_de_pc <= pc;
                    if_de_pc_plus4 <= pc+4;
                end 
     end
     
     
     
//===================== Instruction Decode =====================

    assign utype = {if_de_ir[31:12],12'b0};
    assign itype = {{21{if_de_ir[31]}}, if_de_ir[30:20]};
    assign stype = {{21{if_de_ir[31]}},if_de_ir[30:25],if_de_ir[11:7]};
    assign btype = {{20{if_de_ir[31]}},if_de_ir[7],if_de_ir[30:25],if_de_ir[11:8],1'b0};
    assign jtype = {{12{if_de_ir[31]}},if_de_ir[19:12],if_de_ir[20],if_de_ir[30:21],1'b0};
    
    logic [31:0] ex_rs1, ex_rs2;
    logic cache_stall_plus_one;
    
    CU_DCDR Decoder (
                    .ir_opcode_dcdr(if_de_ir[6:0]),
                    .ir_funct_dcdr(if_de_ir[14:12]),
                    .ir_30_dcdr(if_de_ir[30]),
                    .int_taken_dcdr(0),
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
                    .w_en(mem_wb_reg_write & !cache_stall_plus_one),
                    .adr1(if_de_ir[19:15]),
                    .adr2(if_de_ir[24:20]),
                    .w_adr(mem_wb_rd_addr),
                    .w_data(mem_to_reg_mux_out),
                    .clk(CLK),
                    .rs1(rs1),
                    .rs2(rs2)
                    );
    
    
    
    
    always_ff @(posedge CLK) begin
        cache_stall_plus_one <= cache_stall || data_cache_stall;
    end
    
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
    if (de_jump == 1 && !if_flushed && !if_de_flushed) begin
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
               .ex_mem_stalled(ex_mem_stalled),
               .mem_out_stalled(mem_out_stalled),
               //.cache_stall(cache_stall),
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
    
    logic de_ex_alu_a_sel;
    logic [1:0] de_ex_alu_b_sel;
    logic stalled_for_one;
    
    always_ff @(posedge CLK) begin
    
            // !(de_branch || de_ex_branch) && !stall
            if (!cache_stall && !stall && !data_cache_stall) begin
                // Assign used values
                de_ex_rs1 <= rs1;
                de_ex_rs2 <= rs2;
                de_ex_rs1_addr <= if_de_ir[19:15];
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
                if (if_de_flushed || flush_de_ex) begin
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
                
                de_ex_alu_a_sel <= alu_srca_sel[0];
                de_ex_alu_b_sel <= alu_srcb_sel[1:0];
                
                // Assign immediate values in DE_EX buffer
                de_ex_itype <= itype;
                de_ex_btype <= btype;
                de_ex_jtype <= jtype;
                de_ex_utype <= utype;
                de_ex_stype <= stype;
                
                // Assign register addresses
                de_ex_rd_addr <= if_de_ir[11:7];
                
                
            end
            
            if (stall) begin
                de_ex_stalled <= 1;
            end
            else begin
                de_ex_stalled <= 0;
            end
            
            
            
            
    end
     
    
	
	
//===================== Execute =====================

    assign ex_jump = de_ex_jump;
    wire [31:0] ALU_A, ALU_B;
    
    // Move these into the execute stage
    TwoMux DE_ALU_A_MUX (
                      .RS1(de_ex_rs1),
                      .U_TYPE(de_ex_utype),
                      .ALU_SRC_A(de_ex_alu_a_sel),
                      .SRC_A(ALU_A)
                      );
                      
    FourMux DE_ALU_B_MUX (
                      .ZERO(de_ex_rs2),
                      .ONE(de_ex_itype),
                      .TWO(de_ex_stype),
                      .THREE(de_ex_pc),
                      .SEL(de_ex_alu_b_sel),
                      .OUT(ALU_B)
                      );
     
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
                    .ZERO (ALU_A),
                    .ONE (ex_mem_result),
                    .TWO (mem_to_reg_mux_out),
                    .THREE (0),
                    .SEL (forward_A_SEL),
                    .OUT (forwarded_A)
                    );
    
    FourMux ALU_forward_B (
                    .ZERO (ALU_B),
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
        if(!cache_stall && !data_cache_stall) begin
            ex_mem_stalled <= de_ex_stalled;
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
        end //else ex_mem_memRDEN2 <= 0;
    end



//===================== Memory =====================
    logic [31:0] cache_IOBUS_out, cache_IO_addr_out;
     
    assign mem_RDEN2 = ex_mem_memRDEN2;
    assign mem_WE2 = ex_mem_memWE2;
    assign mem_addr2 = ex_mem_result;
    assign size = ex_mem_size;
    assign sign = ex_mem_sign;
    assign IOBUS_ADDR = cache_IO_addr_out;
    assign IOBUS_OUT = cache_IOBUS_out;
    //assign mem_wb_dout2 = mem_dout2;

    logic [127:0] mem_to_cache, cache_to_mem;

    logic data_hit, dirty, valid, data_miss;
    logic data_update, mem_wb_enable; // data_cache_stall
    
    logic mem_addr_sel;
    logic [31:0] mem_module_addr, mem_wb_addr;

    TwoMux Cache_mem_mux(
                .RS1(mem_addr2),
                .U_TYPE(mem_wb_addr),
                .ALU_SRC_A(mem_addr_sel),
                .SRC_A(mem_module_addr)
                );
    
    OTTER_mem_byte OTTER_MEMORY(
                .MEM_CLK(CLK),
                //.MEM_ADDR1(pc),
                .MEM_ADDR2(mem_module_addr),
                .MEM_DIN2(cache_to_mem),
                .MEM_WRITE2(mem_addr_sel),
                //.MEM_READ1(memRead1 && !stall),
                .MEM_READ2(~mem_addr_sel),
                //.MEM_DOUT1(),
                .MEM_DOUT2(mem_to_cache)
                //.IO_IN(IOBUS_IN),
                //.IO_WR(IOBUS_WR),
                //.MEM_SIZE(size),
                //.MEM_SIGN(sign)
                );


    // Add a mux controlled by the fsm to have the address going into the memory module be for write-back or update

//===================== Stuff for Leb 5 =====================
    Data_Cache_FSM Data_Cache_FSM(
                .CLK(CLK),
                .hit(data_hit),
                .miss(data_miss),
                .dirty_wb(mem_wb_enable),
                .RST(RESET),
                .update(data_update),
                .pc_stall(data_cache_stall),
                .mem_addr_sel(mem_addr_sel)
    );

    logic [31:0] cache_data_out;
    
    Data_Cache Data_Cache(
                .CLK(CLK),
                .write(mem_WE2),
                .read(mem_RDEN2),
                .update(data_update),
                .RESET(RESET),
                .address(mem_addr2),
                .size(size),
                .sign(sign),
                .MM_data_in(mem_to_cache),
                .data_in(ex_mem_rs2),
                .IO_bus_in(IOBUS_IN),
                .IO_bus_out(cache_IOBUS_out),
                .IO_WR(IOBUS_WR),
                .IO_out_addr(cache_IO_addr_out),
                .data_out(cache_data_out),
                .MM_data_out(cache_to_mem),
                .mem_wb_addr(mem_wb_addr),
                .mem_writeback_en(mem_wb_enable),
                .hit(data_hit),
                .dirty(dirty),
                .valid(valid),
                .miss(data_miss)
                );
    
    always_ff @(posedge CLK) begin
        if (!cache_stall && !data_cache_stall) begin
            mem_wb_rd_addr <= ex_mem_rd_addr;
            mem_wb_pc_plus4 <= ex_mem_pc_plus4;
            mem_wb_result <= ex_mem_result;
            mem_wb_dout2 <= cache_data_out;
            
            mem_wb_rf_wr_sel <= ex_mem_rf_wr_sel;
            mem_wb_regWrite <= ex_mem_regWrite;
            mem_out_stalled <= ex_mem_stalled;
        end
    end
 
     
//===================== Write Back =====================

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
            
//===================== L1 CACHE =====================
    wire [31:0] w0, w1, w2, w3, w4, w5, w6, w7, rd;
    wire update, hit, miss;
    imem imem(
        .a(pc),
        .w0(w0),
        .w1(w1),
        .w2(w2),
        .w3(w3),
        .w4(w4),
        .w5(w5),
        .w6(w6),
        .w7(w7)
      );
      
    Cache Cache(
        .PC(pc),
        .CLK(CLK),
        .update(update),
        .w0(w0),
        .w1(w1),
        .w2(w2),
        .w3(w3),
        .w4(w4),
        .w5(w5),
        .w6(w6),
        .w7(w7),
        .rd(IR),
        .hit(hit),
        .miss(miss)
      );
      
    CacheFSM CacheFSM(
        .hit(hit),
        .miss(miss),
        .CLK(CLK),
        .RST(RESET),
        .update(update),
        .pc_stall(cache_stall)
      );     
         
            
            
endmodule