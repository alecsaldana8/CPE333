module Data_Cache ( 
    input logic CLK, 
    input logic write,                // Write enable signal 
    input logic read,                 // Read enable signal 
    input logic update,               // Update signal for handling misses 
    input logic RESET,
    input logic [31:0] address,
    input logic [1:0] size,            // Size to output or input:
    input logic sign,                  // Sign extend? 1:yes, 0:no
    input logic [127:0] MM_data_in,   // Data from main memory (4 words) 
    input logic [31:0] data_in,      // Input data for I/O operations 
    input logic [31:0] IO_bus_in,
    output logic [31:0] IO_bus_out,
    output logic [31:0] IO_out_addr,
    output logic IO_WR,                  // Flag for IO operations
    output logic [31:0] data_out,     // Output data to CPU 
    output logic [127:0] MM_data_out, // Data to main memory (write-back) 
    output logic [31:0] mem_wb_addr,
    output logic mem_writeback_en,            // For write-back
    output logic hit,                 // Hit signal 
    output logic dirty,               // Dirty bit for replacement policy 
    output logic valid,               // Valid bit status
    output logic miss
); 
 
// We don't want to cache IO reads and writes
// Instead, let's check if we are given an address that corresponds to IO
// We can do IO in this module instead as it is simply working with the wrapper

logic [31:0] IO_in_buffer;

// update IO_in_buffer anytime we are reading
// No point in diverting IO to memory module if we can just do it here
always_ff @ (posedge CLK) begin
    if (read) begin
        IO_in_buffer <= IO_bus_in;
    end
end

logic [25:0] tag;
logic [1:0] index, word_offset, byte_offset;

always_comb begin
    tag = address[31:6];
    index = address[5:4];
    word_offset = address[3:2];
    byte_offset = address[1:0];
end

    // Define cache block structure 
    typedef struct { 
        logic [31:0] words[4];  // Four 32-bit words in a block (unpacked array) 
        logic [25:0] tag;       // Tag for address matching 
        logic valid;            // Valid bit 
        logic dirty;            // Dirty bit 
        int   lru;              // LRU counter 
    } cache_block_t; 
 
    // Define cache: 4 sets, 4 blocks per set 
    cache_block_t cache[4][4];  // [sets][blocks] 
 
    // Internal signals 
    logic hit_internal; 
    int hit_block_index; 
    int lru_block_index; 
 
    // Initialization on Reset 
    always_ff @(posedge RESET) begin 
        if (RESET) begin 
            // Initialize cache blocks 
            for (int s = 0; s < 4; s++) begin 
                for (int b = 0; b < 4; b++) begin 
                    cache[s][b].valid <= 0; 
                    cache[s][b].dirty <= 0; 
                    cache[s][b].tag <= 0; 
                    cache[s][b].lru <= 0; 
                    for (int w = 0; w < 4; w++) begin 
                        cache[s][b].words[w] <= 32'b0; 
                    end 
                end 
            end 
        end 
    end 
 
    // Hit Detection Logic
    always_comb begin 
        hit_internal = 0; 
        hit_block_index = -1; 
 
        for (int i = 0; i < 4; i++) begin 
            if (cache[index][i].valid && (cache[index][i].tag == tag)) begin 
                hit_internal = 1; 
                hit_block_index = i; 
                break; 
            end 
        end 
    end 


    // Data Output on Read

    logic [31:0] word_out;

    always_comb begin 
        if (read && address >= 32'h11000000) begin
            data_out = IO_in_buffer;
        end
        else if (read && hit_internal) begin 
             assign word_out = cache[index][hit_block_index].words[word_offset]; 

            // Slice to have proper output
            case({sign,size})
                0: case(byte_offset[1:0])
                        3:  data_out = {{24{cache[index][hit_block_index].words[word_offset][31]}},cache[index][hit_block_index].words[word_offset][31:24]};      // lb  (signed)
                        2:  data_out = {{24{cache[index][hit_block_index].words[word_offset][23]}},cache[index][hit_block_index].words[word_offset][23:16]};
                        1:  data_out = {{24{cache[index][hit_block_index].words[word_offset][15]}},cache[index][hit_block_index].words[word_offset][15:8]};
                        0:  data_out = {{24{cache[index][hit_block_index].words[word_offset][7]}},cache[index][hit_block_index].words[word_offset][7:0]};
                   endcase
                        
                1: case(byte_offset[1:0])
                        2: data_out = {{16{cache[index][hit_block_index].words[word_offset][31]}},cache[index][hit_block_index].words[word_offset][31:16]}; // lh   (signed)
                        1: data_out = {{16{cache[index][hit_block_index].words[word_offset][23]}},cache[index][hit_block_index].words[word_offset][23:8]};
                        0: data_out = {{16{cache[index][hit_block_index].words[word_offset][15]}},cache[index][hit_block_index].words[word_offset][15:0]};
                   endcase

                2: case(byte_offset[1:0])
                        0: data_out = cache[index][hit_block_index].words[word_offset];      // lw     
                   endcase

                4: case(byte_offset[1:0])
                        3:  data_out = {24'd0,cache[index][hit_block_index].words[word_offset][31:24]};      // lbu
                        2:  data_out = {24'd0,cache[index][hit_block_index].words[word_offset][23:16]};
                        1:  data_out = {24'd0,cache[index][hit_block_index].words[word_offset][15:8]};
                        0:  data_out = {24'd0,cache[index][hit_block_index].words[word_offset][7:0]};
                   endcase 

                5: case(byte_offset[1:0])
                        2: data_out = {16'd0,cache[index][hit_block_index].words[word_offset][31:16]};  // lhu
                        1: data_out = {16'd0,cache[index][hit_block_index].words[word_offset][23:8]};
                        0: data_out = {16'd0,cache[index][hit_block_index].words[word_offset][15:0]};
                   endcase
            endcase
    
        end else begin 
            data_out = 32'b0; // Default value if no hit 
        end 
    end 
     
logic [31:0] output_word;

     // Write Logic
always_ff @(posedge CLK) begin 
    IO_WR = 0;
    IO_bus_out = 0;
    IO_out_addr = 0;
    

    if (write && address >= 32'h11000000) begin
        IO_WR = 1;
        IO_bus_out = data_in;
        IO_out_addr = address;
    end
    else if (write && hit_internal) begin 
        cache[index][hit_block_index].dirty <= 1'b1; // Mark block as dirty             data_in; 
        //output_word = cache[index][hit_block_index].words[word_offset];

        // Write specific section (byte, half, word)
        case(size) 
            0:  // Byte
                case(byte_offset)
                    0: cache[index][hit_block_index].words[word_offset] = {{cache[index][hit_block_index].words[word_offset][31:8]},data_in[7:0]};
                    1: cache[index][hit_block_index].words[word_offset] = {{cache[index][hit_block_index].words[word_offset][23:16]},data_in[7:0],{cache[index][hit_block_index].words[word_offset][7:0]}};
                    2: cache[index][hit_block_index].words[word_offset] = {{cache[index][hit_block_index].words[word_offset][31:24]},data_in[7:0], {cache[index][hit_block_index].words[word_offset][15:0]}};
                    3: cache[index][hit_block_index].words[word_offset] = {data_in[7:0],{cache[index][hit_block_index].words[word_offset][23:0]}};
                endcase
            1:  // Halfword
                case(byte_offset)
                    0: cache[index][hit_block_index].words[word_offset] = {{cache[index][hit_block_index].words[word_offset][15:0]},data_in[15:0]};
                    2: cache[index][hit_block_index].words[word_offset] = {data_in[15:0],{cache[index][hit_block_index].words[word_offset][15:0]}};
                endcase

            2:  // Word
                cache[index][hit_block_index].words[word_offset] <= data_in;

            default:
                cache[index][hit_block_index].words[word_offset] <= data_in; // Default to writing a word
        

        endcase


    end 
end 
 
    // Cache Miss Handling (LRU Replacement) 
    always_ff @(posedge update) begin 
        mem_writeback_en = 0;
        if (!hit_internal) begin 
            // Find LRU block to replace 
            lru_block_index = 0; 
            for (int i = 0; i < 4; i++) begin 
                if (!cache[index][i].valid) begin 
                    lru_block_index = i; 
                    break; 
                end else if (cache[index][i].lru < cache[index][lru_block_index].lru) begin 
                    lru_block_index = i; 
                end 
            end 
 
            // Write-back dirty block if necessary 
            if (cache[index][lru_block_index].dirty) begin 
                MM_data_out = {cache[index][lru_block_index].words[0], 
                               cache[index][lru_block_index].words[1], 
                               cache[index][lru_block_index].words[2], 
                               cache[index][lru_block_index].words[3]}; 

                cache[index][lru_block_index].dirty = 0; 
                mem_wb_addr = {cache[index][lru_block_index].tag, index, 4'b0000};
                mem_writeback_en = 1;
            end 
 
            // Replace the LRU block with new data 
            cache[index][lru_block_index].words[0] = MM_data_in[127:96]; 
            cache[index][lru_block_index].words[1] = MM_data_in[95:64]; 
            cache[index][lru_block_index].words[2] = MM_data_in[63:32]; 
            cache[index][lru_block_index].words[3] = MM_data_in[31:0]; 
            cache[index][lru_block_index].tag = tag; 
            cache[index][lru_block_index].valid = 1; 
            cache[index][lru_block_index].lru = 3; // Most recently used 
        end 
 
        // Update LRU counters 
        for (int i = 0; i < 4; i++) begin 
            // Decrement LRU for all except the block we just loaded
            if (i != lru_block_index) cache[index][i].lru -= 1; 
        end 
    end 
 
    // Assign Outputs 
    assign hit = hit_internal; 
    assign miss = (~hit_internal && (read || write));
    assign dirty = hit_internal ? cache[index][hit_block_index].dirty : 0; 
    assign valid = hit_internal ? cache[index][hit_block_index].valid : 0; 
 
endmodule