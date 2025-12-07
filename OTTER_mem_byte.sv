`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: J. Callenes
// 
// Create Date: 01/27/2019 08:37:11 AM
// Design Name: 
// Module Name: bram_dualport
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

//port 1 is read only (instructions - used in fetch stage)
//port 2 is read/write (data - used in writeback stage)
                                                                                                                                //func3
 module OTTER_mem_byte(MEM_CLK,MEM_ADDR1,MEM_ADDR2,MEM_DIN2,MEM_WRITE2,MEM_READ1,MEM_READ2,MEM_DOUT1,MEM_DOUT2,IO_IN,IO_WR,MEM_SIZE,MEM_SIGN);
    parameter ACTUAL_WIDTH=14;  //32KB     16K x 32
    parameter NUM_COL = 4;
    parameter COL_WIDTH = 8;
    
    input [31:0] MEM_ADDR1;     //Instruction Memory Port
    input [31:0] MEM_ADDR2;     //Data Memory Port
    input MEM_CLK;
    input [127:0] MEM_DIN2;
    input MEM_WRITE2;
    input MEM_READ1;
    input MEM_READ2;
    //input [1:0] MEM_BYTE_EN1;
    //input [1:0] MEM_BYTE_EN2;
    input [31:0] IO_IN;
    // output ERR;
    input [1:0] MEM_SIZE;
    input MEM_SIGN;
    output logic [31:0] MEM_DOUT1;
    output logic [127:0] MEM_DOUT2;
    output logic IO_WR;
    
    logic saved_mem_sign;
    logic [1:0] saved_mem_size;
    logic [31:0] saved_mem_addr2;
    
    wire [ACTUAL_WIDTH-1:0] memAddr1,memAddr2;
    logic memWrite2;  
    logic [31:0] memOut2;
    logic [31:0] ioIn_buffer=0;
    logic [NUM_COL-1:0] weA;
   
     assign memAddr1 =MEM_ADDR1[ACTUAL_WIDTH+1:2];
    assign memAddr2 =MEM_ADDR2[ACTUAL_WIDTH+1:2];
    
    (* rom_style="{distributed | block}" *) 
    (* ram_decomp = "power" *) logic [31:0] memory [0:2**ACTUAL_WIDTH-1];
    
//    initial begin
//        $readmemh("Test_All.mem", memory, 0, 2**ACTUAL_WIDTH-1);
//    end 
    
    integer i,j;
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        if(memWrite2)
        begin
            // Write 4 words at a time since we are using cache
            for(j=0; j<4; j=j+1) begin
               memory[memAddr2+j] <= MEM_DIN2[(32*(4-j))-1 -: 32];
            end
         end
        if(MEM_READ2)
            memOut2 <= memory[memAddr2]; 
        //PORT 1  //Instructions
//        if(MEM_READ1)
//            MEM_DOUT1 <= memory[memAddr1];  
            
        saved_mem_size <= MEM_SIZE;
        saved_mem_sign <= MEM_SIGN;
        saved_mem_addr2 <=MEM_ADDR2;
    end
    
//    always_ff @(posedge MEM_CLK)
//        if(MEM_READ2)
//            ioIn_buffer<=IO_IN;       
 
//===  Second cycle ==== Post Processing ==============================
//    logic [31:0] memOut2_sliced=32'b0;
   
//    always_comb
//    begin
//            memOut2_sliced=32'b0;
  
//            case({saved_mem_sign,saved_mem_size})
//                0: case(saved_mem_addr2[1:0])
//                        3:  memOut2_sliced = {{24{memOut2[31]}},memOut2[31:24]};      //lb     //endianess
//                        2:  memOut2_sliced = {{24{memOut2[23]}},memOut2[23:16]};
//                        1:  memOut2_sliced = {{24{memOut2[15]}},memOut2[15:8]};
//                        0:  memOut2_sliced = {{24{memOut2[7]}},memOut2[7:0]};
//                   endcase
                        
//                1: case(saved_mem_addr2[1:0])
//                        //3: memOut2_sliced = {{16{memOut2[31]}},memOut2[31:24]};      //lh   //spans two words, NOT YET SUPPORTED!
//                        2: memOut2_sliced = {{16{memOut2[31]}},memOut2[31:16]};
//                        1: memOut2_sliced = {{16{memOut2[23]}},memOut2[23:8]};
//                        0: memOut2_sliced = {{16{memOut2[15]}},memOut2[15:0]};
//                   endcase
//                2: case(saved_mem_addr2[1:0])
//                        //1: memOut2_sliced = memOut2[31:8];   //spans two words, NOT YET SUPPORTED!
//                        0: memOut2_sliced = memOut2;      //lw     
//                   endcase
//                4: case(saved_mem_addr2[1:0])
//                        3:  memOut2_sliced = {24'd0,memOut2[31:24]};      //lbu
//                        2:  memOut2_sliced = {24'd0,memOut2[23:16]};
//                        1:  memOut2_sliced = {24'd0,memOut2[15:8]};
//                        0:  memOut2_sliced = {24'd0,memOut2[7:0]};
//                   endcase 
//                5: case(saved_mem_addr2[1:0])
//                        //3: memOut2_sliced = {16'd0,memOut2};      //lhu //spans two words, NOT YET SUPPORTED!
//                        2: memOut2_sliced = {16'd0,memOut2[31:16]};
//                        1: memOut2_sliced = {16'd0,memOut2[23:8]};
//                        0: memOut2_sliced = {16'd0,memOut2[15:0]};
//                   endcase
//            endcase
//    end
 
    always_comb begin
        MEM_DOUT2 = {memory[memAddr2], memory[memAddr2 + 1], memory[memAddr2 + 2], memory[memAddr2 + 3]};   
    end 

    always_comb begin
        IO_WR=0;
        if(MEM_ADDR2 >= 32'h11000000)
        begin       
            if(MEM_WRITE2) IO_WR = 1;
            memWrite2=0; 
        end
        else begin 
            memWrite2=MEM_WRITE2;
        end    
    end 
        
 endmodule