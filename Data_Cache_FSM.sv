module Data_Cache_FSM(
    input hit,              // Hit from cache
    input miss,             // Cache miss
    input dirty_wb,          // Block is dirty, signalded by cache and we wb
    input CLK, 
    input RST, 
    output logic update,    // Update signal
    output logic pc_stall,   // Stall signal
    output logic mem_addr_sel     // Selects memory adress in MUX based on wb
                            // 1=wb state, 0=mem read state
);

//Declare States
    typedef enum{
    ST_READ_CACHE,     // Read what is in Cache
    ST_WRITEBACK,      // Write into Cache
    ST_READ_MEM        // Read mem when nothing in cache
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
 always_comb begin // Begin with defaults for FSM
    update=1'b1;
    pc_stall=1'b0;
    mem_addr_sel=1'b0;

    case(PS)
        ST_READ_CACHE:begin
        update=1'b0;
        pc_stall=1'b0; // Still run when hit
        mem_addr_sel=1'b0; // Default to read address from mem
        if(hit)begin
            NS=ST_READ_CACHE;  // Stay in cache to read
        end
        
        else if(miss)begin
            pc_stall=1'b1;  // Stall to get info
            //NS=ST_READ_MEM;

            NS=ST_READ_MEM;  // Else just read mem
        end
        
        else NS=ST_READ_CACHE;
        
       end
       
       ST_WRITEBACK:begin
        update=1'b1;
        pc_stall=1'b1;
        mem_addr_sel=1'b1; // Now we want to read adress from wb state
        
        NS=ST_READ_CACHE;
        end
        
        ST_READ_MEM:begin
            pc_stall=1'b1;
            update=1'b1; 
            mem_addr_sel=1'b0; // Back to the mem read
            if(dirty_wb) NS = ST_WRITEBACK;
            else NS = ST_READ_CACHE;
        end
        
        default:NS=ST_READ_CACHE; // Default state
        
       endcase
       //if (miss) pc_stall = 1;
       //else pc_stall = 0;
    end

endmodule