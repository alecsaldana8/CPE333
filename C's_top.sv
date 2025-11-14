`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 11/02/2023 01:37:36 PM
// Design Name:
// Module Name: top_limMCU
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
LUI = 7'b0110111,
AUIPC = 7'b0010111,
JAL = 7'b1101111,
JALR = 7'b1100111,
BRANCH = 7'b1100011,
LOAD = 7'b0000011,
STORE = 7'b0100011,
OP_IMM = 7'b0010011,
OP = 7'b0110011,
SYSTEM = 7'b1110011

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
logic WE2;
logic RDEN2;
logic regWrite;
logic [1:0] rf_wr_sel;
logic mem_type_sign; //sign, size
logic [1:0] mem_type_size; // should i break up mem type
logic [31:0] pc;
// logic [2:0] pcSource;
logic [1:0] alu_srcA;

logic [2:0] alu_srcB;
logic [31:0] ir;
// logic PCWrite;
// logic RDEN1;
// logic reset DO I ACTUALLY NEED THIS YET
//add cntrl signals from FSM
} instr_t;
typedef struct packed{
logic [31:0] I_type;
logic [31:0] S_type;
logic [31:0] B_type;
logic [31:0] U_type;
logic [31:0] J_type;
} imm;
module OTTER_MCU(
input clk,
input RST,
input intr,
input [31:0] iobus_in,
output [31:0] iobus_out,
output [31:0] iobus_addr,
output iobus_wr
);
//wires
wire [31:0] pc;
wire [31:0] data;
wire [31:0] ir;
wire [31:0] I_type, B_type, J_type, U_type, S_type;
wire [31:0] jal;
wire [31:0] branch;
wire [31:0] jalr;
wire [1:0] rf_wr_sel;
wire [3:0] alu_fun;
wire [1:0] alu_srcA;
wire [2:0] alu_srcB;
wire [1:0] pcSource;
wire [31:0] A,B, rsA, rsB, rs1, rs2;
wire [31:0] dout2, wd_out;
wire stall,stallA, stallB, regWrite, WE2;
wire RDEN2;
wire reset;
// wire ERR;
// wire [31:0] mtvec, mepc, csr_RD;
// wire int_taken, mret_exec, csr_WE, MIE, MPIE;
wire EQ, LT, LTU;
reg [31:0] result;
reg Zero, PCWrite, RDEN1, flush;
wire [1:0] FwrdSelA, FwrdSelB;
//linked input-output signals
assign iobus_addr = result;
assign iobus_out = rs2;

assign reset = RST;
//==== Instruction Fetch ===========================================
assign RDEN1 = 1'b1;
logic [31:0] if_de_pc;
//PC MUX
mux_4t1_nb #(.n(32)) PC_MUX(
.SEL (pcSource),
.D0 (pc + 4),
.D1 (jalr),
.D2 (branch),
.D3 (jal),
// .D4 (mtvec),
// .D5 (mepc),
.D_OUT (data)
);
//PC register
reg_nb_sclr #(.n(32)) REG(
.data_in (data),
.clk (clk),
.clr (reset),
.ld (PCWrite), //put strcut version?? but i dont have one for
fetch
.data_out (pc)
);
always_comb begin
if (stall == 1'b0) begin
PCWrite <= 1'b1;
end
else begin
PCWrite <= 1'b0;
end
end
//==== Instruction Decode ===========================================
logic [31:0] de_ex_opA;
logic [31:0] de_ex_opB;
logic [31:0] de_ex_rs1;
logic [31:0] de_ex_rs2;
logic [31:0] if_de_ir;
logic [31:0] ex_mem_aluRes;
logic [31:0] wb_dout2;
logic stalledA;
logic stalledB;

instr_t decode_instr, execute_instr, mem_instr, wb_instr;
imm decode_imm, execute_imm, mem_imm, wb_imm;

// i dont know what this does
opcode_t OPCODE;
//whole lotta assigns

assign OPCODE = opcode_t'(if_de_ir[6:0]);
assign decode_instr.rs1_addr=if_de_ir[19:15];
assign decode_instr.rs2_addr=if_de_ir[24:20];
assign decode_instr.rd_addr=if_de_ir[11:7];
assign decode_instr.opcode=OPCODE;
assign decode_instr.alu_fun = alu_fun;
assign decode_instr.WE2 = WE2;
assign decode_instr.RDEN2 = RDEN2;
assign decode_instr.regWrite = regWrite;
assign decode_instr.rf_wr_sel = rf_wr_sel;
assign decode_instr.mem_type_sign = if_de_ir[14]; //sign, size
assign decode_instr.mem_type_size = if_de_ir[13:12];
assign decode_instr.pc = if_de_pc;
// assign decode_instr.pcSource = PCSource;
assign decode_instr.alu_srcA = alu_srcA;
assign decode_instr.alu_srcB = alu_srcB;
assign decode_instr.ir = if_de_ir;
// imm
assign decode_imm.I_type = I_type;
assign decode_imm.J_type = J_type;
assign decode_imm.B_type = B_type;
assign decode_imm.U_type = U_type;
assign decode_imm.S_type = S_type;
//used register signals
assign decode_instr.rs1_used = decode_instr.rs1_addr != 0
&& decode_instr.opcode != LUI
&& decode_instr.opcode != AUIPC
&& decode_instr.opcode != JAL;
// && stallA != 1'b1;;
assign decode_instr.rs2_used = decode_instr.rs2_addr != 0
&& decode_instr.opcode != OP_IMM
&& decode_instr.opcode != LUI
&& decode_instr.opcode != AUIPC
&& decode_instr.opcode != JAL;

// && stallB != 1'b1;
assign decode_instr.rd_used = decode_instr.rd_addr != 0
&& decode_instr.opcode != BRANCH
&& decode_instr.opcode != STORE;

always_ff@(posedge clk) begin
if (flush) begin

if_de_ir <= 0;
if_de_pc <= 0;
end
else if (stall == 0 ) begin
if_de_ir <= ir;
if_de_pc <= pc;
end
end
CU_DCDR my_cu_dcdr(
// .br_eq (EQ),
// .br_lt (LT),
// .br_ltu (LTU),
.opcode (OPCODE),
.func7 (if_de_ir[30]),
.func3 (if_de_ir[14:12]),
.INT_TAKEN (0),
.alu_fun (alu_fun),
// .pcSource (PCSource),
.alu_srcA (alu_srcA),
.alu_srcB (alu_srcB),
.rf_wr_sel (rf_wr_sel), //reg MUX select
.pcWrite (), //no need for struct?
.regWrite (regWrite),
.memWE2 (WE2),
.memRDEN1 (),
.memRDEN2 (decode_instr.RDEN2)
);

// Forwarding Unit (im detecting hazards a cycle early)
ForwardingUnit my_FU (
.rs1 (decode_instr.rs1_addr),
.rs2 (decode_instr.rs2_addr),
.rs1_used (decode_instr.rs1_used),
.rs2_used (decode_instr.rs2_used),
.rd_used (execute_instr.rd_used), // or decode?
.rd_used_mem (mem_instr.rd_used),
.stalledA (stalledA),
.stalledB (stalledB),
.ExRd (execute_instr.rd_addr),
.MemRd (mem_instr.rd_addr),
.ExRegWrite (execute_instr.regWrite),
.MemRegWrite (mem_instr.regWrite),
.RDEN2 (execute_instr.RDEN2),
.pcSource (pcSource), // i dont think flush will be long enough
.FwrdMuxA (FwrdSelA),
.FwrdMuxB (FwrdSelB),
.stall (stall),
.stallB (stallB),
.stallA (stallA),
.flush (flush)
);

//Fwrd Reg A
mux_4t1_nb #(.n(32)) my_muxFwrdA(
.SEL (FwrdSelA),
.D0 (rs1),
.D1 (result),
.D2 (ex_mem_aluRes),
.D3 (dout2),
.D_OUT (rsA) );
//Fwrd Reg B
mux_4t1_nb #(.n(32)) my_muxFwrdB(
.SEL (FwrdSelB),
.D0 (rs2),
.D1 (result),
.D2 (ex_mem_aluRes),
.D3 (dout2),
.D_OUT (rsB) );
//alu A MUX (2 input)
mux_4t1_nb #(.n(32)) my_muxA(
.SEL (decode_instr.alu_srcA),
.D0 (rsA),
.D1 (U_type),
.D2 (~rsA),
.D3 (0),
.D_OUT (de_ex_opA) );
//alu B MUX
mux_8t1_nb #(.n(32)) my_muxB(
.SEL (decode_instr.alu_srcB),
.D0 (rsB),
.D1 (I_type),
.D2 (S_type),
.D3 (decode_instr.pc),
//.D4 (csr_RD),
.D_OUT (de_ex_opB) );
//imm gen module
assign I_type = {{21{if_de_ir[31]}}, if_de_ir[30:25], if_de_ir[24:20]};
assign S_type = {{21{if_de_ir[31]}}, if_de_ir[30:25], if_de_ir[11:7]};
assign B_type = {{20{if_de_ir[31]}}, if_de_ir[7], if_de_ir[30:25],
if_de_ir[11:8], 1'b0};
assign U_type = {if_de_ir[31:12], 12'h000};
assign J_type = {{12{if_de_ir[31]}}, if_de_ir[19:12], if_de_ir[20],
if_de_ir[30:21], 1'b0};
//==== Execute ======================================================
logic [31:0] ex_mem_rs2;
logic [31:0] ex_mem_opA;
logic [31:0] ex_mem_opB;
logic [31:0] ex_rsB;

logic [31:0] ex_ir;
always_ff@(posedge clk) begin
if (flush) begin
execute_instr <= 0;
execute_imm <= 0;
de_ex_rs2 <= 0;
de_ex_rs1 <= 0;
ex_rsB<=0;
stalledA <= 0;
stalledB <= 0;
end
else if (stall == 0 ) begin
execute_instr <= decode_instr;
execute_imm <= decode_imm;
de_ex_rs2 <= rs2; //should i do this???
de_ex_rs1 <= rs1;
ex_rsB<=rsB;
stalledA <= 0;
stalledB <= 0;
ex_ir<=if_de_ir;
end
else begin
execute_instr.RDEN2 <= decode_instr.RDEN2;
execute_instr.regWrite <= 0;
stalledA <= stallA;
stalledB <= stallB;
end
end

// Creates a RISC-V ALU
// the ALU
ALU my_alu(
.A (ex_mem_opA),
.B (ex_mem_opB),
.alu_fun (execute_instr.alu_fun), //execute_instr??
.result (result),
.Zero (Zero)
);
// branch cond generator
// it only outputs pcSpurce
BCG my_bcg(
.func3 (ex_ir[14:12]),
.opcode (execute_instr.opcode),
.rs1 (ex_mem_opA), // ex or dec?
.rs2 (ex_mem_opB),
// .br_eq (EQ),
// .br_lt (LT),
// .br_ltu (LTU),
.pcSource(pcSource)
);

//branch module
assign jal = execute_instr.pc + execute_imm.J_type;
assign jalr = execute_imm.I_type + ex_mem_opA;
assign branch = execute_instr.pc + execute_imm.B_type;

//==== Memory ======================================================
assign iobus_addr = ex_mem_aluRes;
assign iobus_out = ex_mem_opB;
logic [31:0] mem_wb_opB;
logic [31:0] mem_rsB;
always_ff@(posedge clk) begin
mem_wb_opB<=ex_mem_opB;
mem_instr <= execute_instr;
mem_imm <= execute_imm;
ex_mem_rs2 <= de_ex_rs2;
wb_dout2 <= dout2;
mem_rsB<=ex_rsB;
ex_mem_aluRes <= result; //weird
ex_mem_opA <= de_ex_opA;
ex_mem_opB <= de_ex_opB;
end

//memory
Memory OTTER_MEMORY (
.MEM_CLK (clk),
.MEM_RDEN1 (RDEN1),
.MEM_RDEN2 (mem_instr.RDEN2),
.MEM_WE2 (mem_instr.WE2), //should be mem ...
.MEM_ADDR1 (pc[15:2]), //14 bit signal
.MEM_ADDR2 (ex_mem_aluRes),
.MEM_DIN2 (mem_rsB),
.MEM_SIZE (mem_instr.mem_type_size), //ir [13:12
.MEM_SIGN (mem_instr.mem_type_sign), //ir[14]
.IO_IN (iobus_in),
.IO_WR (iobus_wr), //unused
.MEM_DOUT1 (ir),
.MEM_DOUT2 (dout2) );
//==== Write Back ==================================================
logic [31:0] mem_wb_aluRes;
always_ff@(posedge clk) begin
wb_instr <= mem_instr;
wb_imm <= mem_imm;
mem_wb_aluRes <= ex_mem_aluRes;
end
//REG FILE MUX
mux_4t1_nb #(.n(32)) my_muxREG(

.SEL (wb_instr.rf_wr_sel),
.D0 (wb_instr.pc + 4),
.D1 (csr_RD),
.D2 (wb_dout2),
.D3 (mem_wb_aluRes),
.D_OUT (wd_out) );
//REG FILE
RegFile my_regfile (
.wd (wd_out),
.clk (clk),
.en (wb_instr.regWrite),
.adr1 (decode_instr.rs1_addr),
.adr2 (decode_instr.rs2_addr),
.wa (wb_instr.rd_addr), //should this come from the wb struct?
.rs1 (rs1),
.rs2 (rs2)
);
endmodule