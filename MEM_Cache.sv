`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/04/2024 12:18:44 PM
// Design Name: 
// Module Name: MEM_Cache
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

//32-bit Address
//
// Tag = [31:6]
// Index = [5:4]
// Word Offset = [3:2] 
// Byte Offset = [1:0]

module MEM_Cache(
         input [31:0] addr,
    input [31:0] in_data,
    input CLK, RST,
//    input update,
    
    input MEM_READ,
    input MEM_WRITE,
    
    input [1:0] MEM_SIZE,
    input MEM_SIGN,
    input [31:0]IOBUS_IN,
    input IOBUS_WR,
    
//    input logic [31:0] w0,input logic [31:0] w1,
//    input logic [31:0] w2, input logic [31:0] w3,
//    input logic [31:0] w4, input logic [31:0] w5,
//    input logic [31:0] w6, input logic [31:0] w7,
    output logic [31:0] DOUT2,
    
//    output logic hit, 
//    output logic miss,
    
    output CACHE_stall
    );
    
parameter SA_WAYS = 4;
parameter NUM_BLOCKS = 4;
parameter BLOCK_SIZE = 4;
parameter INDEX_SIZE = 2;
parameter WORD_OFFSET_SIZE = 2;
parameter BYTE_OFFSET = 2;
parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET; //31 - 2 - 2 - 2

logic [31:0]            data[SA_WAYS-1:0][NUM_BLOCKS-1:0][BLOCK_SIZE-1:0];  //data[sa_offset][index][word_offset][byte_offset +: 8]
logic [TAG_SIZE-1:0]    tags[SA_WAYS-1:0][NUM_BLOCKS-1:0];
logic                   valid_bits[SA_WAYS-1:0][NUM_BLOCKS-1:0];
logic                   dirty_bits[SA_WAYS-1:0][NUM_BLOCKS-1:0];

logic [31:0] l2_out, l2_in;
logic [31:0] l2_addr_rd, l2_addr_wr, l2_addr;
logic l2_addr_sel;

logic [31:0] word_string [3:0];
int word_count;
int wb_i = 0;

logic FSM_WRITE, FSM_READ;
logic load, overwrite, set_dirty;

logic [1:0] index, word_offset, byte_offset, sa_offset;
logic [TAG_SIZE-1:0]cache_tag, in_tag;
//logic [2:0] pc_offset;

logic [31:0] out_data, memOut2_sliced;
logic [1:0] U_bit, counter;


//add extra for loop so setting each WAY
initial begin
    int i;int j; int sa;
    for(sa = 0; sa < SA_WAYS; sa = sa + 1) begin
        for(i = 0; i < NUM_BLOCKS; i = i + 1) begin //initializing RAM to 0
            for(j=0; j < BLOCK_SIZE; j = j + 1)
                data[sa][i][j] = 32'b0;
            tags[sa][i] = 32'b0;
            valid_bits[sa][i] = 1'b0;
            dirty_bits[sa][i] = 1'b0;
        end
    end
end

//assign index = PC[8:5];
//assign validity = valid_bits[index];
//assign cache_tag = tags[index];
//assign pc_offset = PC[4:2];
//assign pc_tag = PC[31:9];

assign byte_offset = addr[1:0];
assign word_offset = addr[3:2];
assign index = addr[5:4];
assign in_tag = addr[31:6];



//calculate hit logic for all blocks in index
//hit variable ORs all 

assign hit0 = (valid_bits[0][index] && (tags[0][index] == in_tag));
assign hit1 = (valid_bits[1][index] && (tags[1][index] == in_tag));
assign hit2 = (valid_bits[2][index] && (tags[2][index] == in_tag));
assign hit3 = (valid_bits[3][index] && (tags[3][index] == in_tag));


assign hit = hit0 | hit1 | hit2 | hit3;
assign miss = !hit;

always_comb begin
    if(hit0) sa_offset = 2'b00;
    else if(hit1) sa_offset = 2'b01;
    else if(hit2) sa_offset = 2'b10;
    else if(hit3) sa_offset = 2'b11;
    
end

//always_comb begin
//    rd = 32'h00000013; //nop
//    if(hit) rd = data[index][pc_offset];
//end

always_ff @(posedge CLK) begin
    //if it hits & read, loads the proper data on the clock cycle
    if(MEM_READ) begin
        if(hit) begin
            if(hit0) begin
                out_data <= data[0][index][word_offset];
                U_bit[1] <= 'b1; 
                U_bit[0] <= 'b1;
            end
            else if(hit1) begin
                out_data <= data[1][index][word_offset];
                U_bit[1] <= 'b1;
                U_bit[0] <= 'b0; 
            end
            else if(hit2) begin
                out_data <= data[2][index][word_offset];
                U_bit[1] <= 'b0;
                U_bit[0] <= 'b1; 
            end
            else if(hit3) begin
                out_data <= data[0][index][word_offset];
                U_bit[1] <= 'b0;
                U_bit[0] <= 'b0; 
            end
            
        end
        //logic for miss!!
        // What do we do?
        
        //Get data from l2 memory
            //Set mem_size to 'b10
            //set read_en to '1
            //store resulting 4 words in an array
            
         //decide which SA_Way to replace using LRU and U_Bit
         //if dirty bit is 1, write data to Memory
            // this is going to stall for 4 clock cycles
         //after writing, overwrite data in cache
//        else begin
////            load = 'b1;
//        end
            
    end
 end
 
 //format output
 //takes care of LB, LBU, LH, LHU
 always_comb
    begin
            memOut2_sliced=32'b0;
  
            case({MEM_SIGN, MEM_SIZE})
                0: case(addr[1:0])
                        3:  memOut2_sliced = {{24{out_data[31]}},out_data[31:24]};      //lb     //endianess
                        2:  memOut2_sliced = {{24{out_data[23]}},out_data[23:16]};
                        1:  memOut2_sliced = {{24{out_data[15]}},out_data[15:8]};
                        0:  memOut2_sliced = {{24{out_data[7]}},out_data[7:0]};
                   endcase
                        
                1: case(addr[1:0])
                        3: memOut2_sliced = {{16{out_data[31]}},out_data[31:24]};      //lh   //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {{16{out_data[31]}},out_data[31:16]};
                        1: memOut2_sliced = {{16{out_data[23]}},out_data[23:8]};
                        0: memOut2_sliced = {{16{out_data[15]}},out_data[15:0]};
                   endcase
                2: case(addr[1:0])
                        1: memOut2_sliced = out_data[31:8];   //spans two words, NOT YET SUPPORTED!
                        0: begin 
                            memOut2_sliced = out_data;      //lw  
//                            w1_out = memOut_w1;  
                           end 
                   endcase
                4: case(addr[1:0])
                        3:  memOut2_sliced = {24'd0,out_data[31:24]};      //lbu
                        2:  memOut2_sliced = {24'd0,out_data[23:16]};
                        1:  memOut2_sliced = {24'd0,out_data[15:8]};
                        0:  memOut2_sliced = {24'd0,out_data[7:0]};
                   endcase
                5: case(addr[1:0])
                        3: memOut2_sliced = {16'd0,out_data};      //lhu //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {16'd0,out_data[31:16]};
                        1: memOut2_sliced = {16'd0,out_data[23:8]};
                        0: memOut2_sliced = {16'd0,out_data[15:0]};
                   endcase
            endcase
            
            DOUT2 = memOut2_sliced;
    end
 
 // Deals with Writing
 //if write en is high and there is a hit, update the cache and set dirty bit to 1
 // only words are read/written to L2 Memory
 // deal with LBU and LHU in Cache
 always_ff @(posedge CLK) begin
    l2_addr <= 31'b0;
    
    if(MEM_WRITE) begin
        //FSM Does nothing, keeps on going
        if(hit) begin
            case(MEM_SIZE)
                0: data[sa_offset][index][word_offset][byte_offset*8 +: 8] <= in_data[7:0]; //MEM_DIN2[(3-i)*COL_WIDTH +: COL_WIDTH]; //
                1: data[sa_offset][index][word_offset][byte_offset*8 +: 16] <= in_data[15:0];     
                2: data[sa_offset][index][word_offset] <= in_data[31:0];
                default: data[sa_offset][index][word_offset] <= in_data[31:0];
            endcase
            
            //set dirty bit
            if(set_dirty)
                dirty_bits[sa_offset][index] <= 'b1; 
//            load = 'b0;    
        end
        //FSM needs to stall >= 2 stages
            //1 CC: write data to l2
            //2 CC: load data into cache
        //Writes data to L2
        //redirects to load cache
        else if (miss) begin
            //FSM set MEM_WRITE2 to high
            l2_addr <= addr;
            l2_in <= in_data;
//            load ='b1;
        end
    end
//end

////loads data from Main Memory into Cache
//always_ff @(posedge CLK) begin
    if(load) begin
    //FSM set MEM_read to 1
        
        //if index position is full
        if(valid_bits[0][index] && valid_bits[1][index] && valid_bits[2][index] && valid_bits[3][index]) begin
        
//            overwrite = 'b1;
//            load = 'b1;
           //set SA_offset to current U_bit
           sa_offset <= U_bit;
           
           if(dirty_bits[sa_offset][index]) begin
                //write old data to memory
                
                //set MEM_WRITE to 1
                //set l2_addr_sel to 1
                //set MEM_SIZE to 'b10 (word)
                //WB stage must repeat 4 times
                if(writeback == 'b1) begin
                    l2_in <= data[sa_offset][index][counter];
                    l2_addr <= {tags[sa_offset][index], index, 4'b0};
                    
//                    wb_i <= wb_i + 1;
                end
           
                //finished writeback, sets
                else if(writeback == 'b0) begin
                    l2_addr <= addr;
//                    wb_i <= 0;
                
                    data[sa_offset][index][0] <= word_string[0];
                    data[sa_offset][index][1] <= word_string[1];
                    data[sa_offset][index][2] <= word_string[2];
                    data[sa_offset][index][3] <= word_string[3];
                    
                    tags[sa_offset][index] <= in_tag;
                    valid_bits[sa_offset][index] <= 1'b1;
                    dirty_bits[sa_offset][index] <= 'b0;
                end
           
           end 
           //current data is not dirty (data matches the L2)
           else begin
                data[sa_offset][index][0] <= word_string[0];
                data[sa_offset][index][1] <= word_string[1];
                data[sa_offset][index][2] <= word_string[2];
                data[sa_offset][index][3] <= word_string[3];
                
                tags[sa_offset][index] <= in_tag;
                valid_bits[sa_offset][index] <= 1'b1;

           end
        end
        
        //there is an open slot to put data
        else begin
        
            //logic to find open slot
            if(valid_bits[0][index] == 'b0) begin
                sa_offset <= 'b00;
            end else if(valid_bits[1][index] == 'b0) begin
                sa_offset <= 2'b01;
            end else if(valid_bits[2][index] == 'b0) begin 
                sa_offset <= 2'b10;
            end else if(valid_bits[3][index] == 'b0) begin
                sa_offset <= 2'b11;
            end
        
        
            //writes data to cache
            data[sa_offset][index][0] <= word_string[0];
            data[sa_offset][index][1] <= word_string[1];
            data[sa_offset][index][2] <= word_string[2];
            data[sa_offset][index][3] <= word_string[3];
            
            tags[sa_offset][index] <= in_tag;
            valid_bits[sa_offset][index] <= 1'b1;
        
        end //end "open" else
    end

end


//TwoMux Memory_Address (.SEL(writeback), .A(l2_addr_rd), .B(l2_addr_wr), .OUT(l2_addr));

 OTTER_mem_byte OTTER_MEMORY(.MEM_CLK(CLK), .MEM_READ2(FSM_READ), 
        .MEM_WRITE2(FSM_WRITE), .MEM_r0(word_string[0]), .MEM_r1(word_string[1]), .MEM_r2(word_string[2]), .MEM_r3(word_string[3]), //reads 4 words at a time
        .MEM_ADDR2(l2_addr), .MEM_DIN2(l2_in), .MEM_SIZE(MEM_SIZE),
         .MEM_SIGN('b0), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR),  .MEM_DOUT2(l2_out));

MEM_FSM MEM_FSM(.CLK(CLK), .RST(RST), .dirty(dirty_bits[sa_offset][index]),
    .hit(hit), .miss(miss), .read(MEM_READ), .write(MEM_WRITE), .load(load), .writeback(writeback), 
    .FSM_Write(FSM_WRITE), .FSM_Read(FSM_READ), .stall(CACHE_stall), .set_dirty(set_dirty), .counter(counter));


always_ff @(negedge CLK) begin
    //might be able to output all 4 words at once
    //adjust address by 4
    
    //=============================================================================
    //FSM NEEDS TO STALL FOR 4 CLOCK CYCLES when WB
    //
    //=============================================================================
//    for(word_count = 0; word_count < BLOCK_SIZE; word_count = word_count + 1) begin
//        l2_addr_rd = addr[31:4] + i<<2;
//        word_string[word_count] == l2_out;
//    end
end




endmodule
