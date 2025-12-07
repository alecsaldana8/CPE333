`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/26/2025 04:30:49 PM
// Design Name: 
// Module Name: CacheFSM
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


module CacheFSM(
    input hit, 
    input miss, 
    input CLK, 
    input RST, 
    output logic update, 
    output logic pc_stall
);

//Declare States
    typedef enum{
    ST_READ_CACHE,
    ST_READ_MEM 
//    ST_WAIT_MEM,
//    ST_UPDATE_CACHE
    } state_type;

//Current and Next State
state_type PS, NS;

//State Switch on Clock Edge
always_ff @(posedge CLK) begin
    if(RST == 1)
        PS <= ST_READ_MEM;
    else
        PS <= NS;
end

//FSM Logic
 always_comb begin
    update=1'b1;
    pc_stall=1'b0;
    
    case(PS)
        ST_READ_CACHE:begin
        update=1'b0;
        if(hit)begin
            NS=ST_READ_CACHE;
        end
        
        else if(miss)begin
            pc_stall=1'b1;
            NS=ST_READ_MEM;
        end
        
        else NS=ST_READ_CACHE;
        
       end
        
        ST_READ_MEM:begin
            pc_stall=1'b1;
            NS=ST_READ_CACHE;
        end
        
        default:NS=ST_READ_CACHE;
       endcase
       //if (miss) pc_stall = 1;
       //else pc_stall = 0;
    end
//endmodule

//always_comb begin
//    update = 1'b1;
//    pc_stall = 0;
    
//    case (PS)
//        ST_READ_CACHE: begin
//        update = 1'b0;
//        if(hit) begin
//            NS = ST_READ_CACHE;
//        end
//        else if(miss) begin
//            pc_stall = 1'b1;
//            NS = ST_READ_MEM;
//        end

//     else NS = ST_READ_CACHE;
//    end

//    ST_READ_MEM: begin
//     update = 1'b0;
//     pc_stall = 1'b1;
//     NS = ST_WAIT_MEM;
//   end
   
//    ST_WAIT_MEM: begin
//     update = 1'b0;
//     pc_stall = 1'b1;
//     NS = ST_UPDATE_CACHE;
//   end
   
//    ST_UPDATE_CACHE: begin
//     update = 1'b1;
//     pc_stall = 1'b0;
//     NS = ST_READ_CACHE;
//   end
   
//  default: NS = ST_READ_CACHE;
//  endcase
// end

endmodule