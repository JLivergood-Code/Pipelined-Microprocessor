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
module OTTER_mem_dualport(MEM_CLK,MEM_ADDR1,MEM_ADDR2,MEM_DIN2,MEM_WRITE2,MEM_READ1,MEM_READ2,ERR,MEM_DOUT1,MEM_DOUT2,
    MEM_w0, MEM_w1, MEM_w2, MEM_w3, IO_IN,IO_WR);
    parameter ACTUAL_WIDTH=14;  //32KB
    //parameter BYTES_PER_WORD;
    input [31:0] MEM_ADDR1;     //Instruction Memory Port
    input [31:0] MEM_ADDR2;     //Data Memory Port
    input MEM_CLK;
    input [31:0] MEM_DIN2;
    input logic [31:0] MEM_w0;
    input logic [31:0] MEM_w1; 
    input logic [31:0] MEM_w2;
    input logic [31:0] MEM_w3;
    input MEM_WRITE2;
    input MEM_READ1;
    input MEM_READ2;
    //input [1:0] MEM_BYTE_EN1;
    //input [1:0] MEM_BYTE_EN2;
    input [31:0] IO_IN;
    output ERR;
    //input [1:0] MEM_SIZE;
    output logic [31:0] MEM_DOUT1;
    output logic [31:0] MEM_DOUT2;
    output logic [31:0] MEM_r0;
    output logic [31:0] MEM_r1; 
    output logic [31:0] MEM_r2;
    output logic [31:0] MEM_r3;
    output logic IO_WR;
    
    wire [ACTUAL_WIDTH-1:0] memAddr1,memAddr2;
    logic memWrite2;  
    logic [31:0] memOut2;
    logic [31:0] memOut_w0, memOut_w1, memOut_w2, memOut_w3;

    logic [31:0] ioIn_buffer=0;
    
    assign memAddr1 =MEM_ADDR1[ACTUAL_WIDTH+1:2];
    assign memAddr2 =MEM_ADDR2[ACTUAL_WIDTH+1:2];
    
    (* rom_style="{distributed | block}" *)
   logic [31:0] memory [0:2**ACTUAL_WIDTH-1];
    
    initial begin
        $readmemh("otter_memory.mem", memory, 0, 2**ACTUAL_WIDTH-1);
    end 
    
    
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        //=======================================================================================
        //DOES NOT WORK!!! NEEDS TO WRITE HW AND B TO MEMORY IN CASE OF CACHE MISS!! ============
        //=======================================================================================
        //writes only with words
        // all half word and byte calculation needs to be done in Cache
        
        if(memWrite2)
            memory[memAddr2]    <= MEM_w0;
            memory[memAddr2+4]  <= MEM_w1;
            memory[memAddr2+8]  <= MEM_w2;
            memory[memAddr2+12] <= MEM_w3;
        if(MEM_READ2)
          memOut_r0 <= memory[memAddr2];
          memOut_r1 <= memory[memAddr2+4];
          memOut_r2 <= memory[memAddr2+8];
          memOut_r3 <= memory[memAddr2+12];
        //PORT 1  //Instructions
//        if(MEM_READ1)
//            MEM_DOUT1 <= memory[memAddr1];  
    end
    
    //Check for misalligned or out of bounds memory accesses
    assign ERR = ((MEM_ADDR1 >= 2**ACTUAL_WIDTH)|| (MEM_ADDR2 >= 2**ACTUAL_WIDTH)
                    || MEM_ADDR1[1:0] != 2'b0 || MEM_ADDR2[1:0] !=2'b0)? 1 : 0; 
            
    
    always_ff @(posedge MEM_CLK)
        if(MEM_READ2)
            ioIn_buffer<=IO_IN;       
 
 
    always_comb
    begin
        IO_WR=0;
        if(MEM_ADDR2 >= 32'h11000000)
        begin       
            if(MEM_WRITE2) IO_WR = 1;
            memWrite2=0;
            MEM_DOUT2 = ioIn_buffer;  
        end
        else begin 
            memWrite2=MEM_WRITE2;
            MEM_DOUT2 = memOut_r0;
            MEM_r0 = memOut_r0;
            MEM_r0 = memOut_r1;
            MEM_r0 = memOut_r2;
            MEM_r0 = memOut_r3;
        end    
    end 
        
 endmodule
                                                                                                                                //func3
 module OTTER_mem_byte(MEM_CLK,MEM_ADDR1,MEM_ADDR2,MEM_DIN2,MEM_WRITE2,MEM_READ1,MEM_READ2,ERR,MEM_DOUT1,
    MEM_w0, MEM_w1, MEM_w2, MEM_w3, MEM_DOUT2,IO_IN,IO_WR,MEM_SIZE,MEM_SIGN);
    parameter ACTUAL_WIDTH=14;  //32KB     16K x 32
    parameter NUM_COL = 4;
    parameter COL_WIDTH = 8;
    
    input [31:0] MEM_ADDR1;     //Instruction Memory Port
    input [31:0] MEM_ADDR2;     //Data Memory Port
    input MEM_CLK;
    input [31:0] MEM_DIN2;
    input MEM_WRITE2;
    input MEM_READ1;
    input MEM_READ2;
    input logic [31:0] MEM_r0;
    input logic [31:0] MEM_r1; 
    input logic [31:0] MEM_r2;
    input logic [31:0] MEM_r3;
    //input [1:0] MEM_BYTE_EN1;
    //input [1:0] MEM_BYTE_EN2;
    input [31:0] IO_IN;
    output ERR;
    input [1:0] MEM_SIZE;
    input MEM_SIGN;
    output logic [31:0] MEM_DOUT1;
    output logic [31:0] MEM_DOUT2;
    output logic IO_WR;
    
    logic saved_mem_sign;
    logic [1:0] saved_mem_size;
    logic [31:0] saved_mem_addr2;
    
    wire [ACTUAL_WIDTH-1:0] memAddr1,memAddr2;
    logic memWrite2;  
    logic [31:0] memOut_w0, memOut_w1, memOut_w2, memOut_w3;
    logic [31:0] ioIn_buffer=0;
    logic [NUM_COL-1:0] weA;
   
     assign memAddr1 =MEM_ADDR1[ACTUAL_WIDTH+1:2];
    assign memAddr2 =MEM_ADDR2[ACTUAL_WIDTH+1:2];
    
    (* rom_style="{distributed | block}" *) 
    (* ram_decomp = "power" *) logic [31:0] memory [0:2**ACTUAL_WIDTH-1];
    
    initial begin
        $readmemh("testall.mem", memory, 0, 2**ACTUAL_WIDTH-1);
    end 
    

    always_comb
    begin
        case(MEM_SIZE)
                0:  weA = 4'b1 << MEM_ADDR2[1:0];   //sb
                1:  weA =4'b0011 << MEM_ADDR2[1:0];  //sh      //Not supported if across word boundary
                2:  weA=4'b1111;                    //sw        //Not supported if across word boundary
                default: weA=4'b0000;
        endcase
    end
    integer i,j;
    always_ff @(posedge MEM_CLK) begin
        //PORT 2  //Data
        if(memWrite2)
        begin
            j=0;
            for(i=0;i<NUM_COL;i=i+1) begin
                if(weA[i]) begin
                        case(MEM_SIZE)
                            0: memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[7:0]; //MEM_DIN2[(3-i)*COL_WIDTH +: COL_WIDTH];
                            1: begin 
                                    memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[j*COL_WIDTH +: COL_WIDTH];
                                    j=j+1;
                               end
                            2: memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[i*COL_WIDTH +: COL_WIDTH];
                            default:  memory[memAddr2][i*COL_WIDTH +: COL_WIDTH] <= MEM_DIN2[i*COL_WIDTH +: COL_WIDTH];
                        endcase
                end
            end
         end
        if(MEM_READ2)
            memOut_w0 <= memory[memAddr2];
            memOut_w1 <= memory[memAddr2+4];
            memOut_w2 <= memory[memAddr2+8];
            memOut_w3 <= memory[memAddr2+12];
            
            ioIn_buffer<=IO_IN; 
        //PORT 1  //Instructions
//        if(MEM_READ1)
//            MEM_DOUT1 <= memory[memAddr1];  
            
        saved_mem_size <= MEM_SIZE;
        saved_mem_sign <= MEM_SIGN;
        saved_mem_addr2 <=MEM_ADDR2;
    end
    
    //Check for misalligned or out of bounds memory accesses
    assign ERR = ((MEM_ADDR1 >= 2**ACTUAL_WIDTH)|| (MEM_ADDR2 >= 2**ACTUAL_WIDTH)
                    || MEM_ADDR1[1:0] != 2'b0 || MEM_ADDR2[1:0] !=2'b0)? 1 : 0; 
            
    
//    always_ff @(posedge MEM_CLK)
//        if(MEM_READ2)
                   
 
//===  Second cycle ==== Post Processing ==============================
    logic [31:0] memOut2_sliced=32'b0;
   
    always_comb
    begin
            memOut2_sliced=32'b0;
  
            case({saved_mem_sign,saved_mem_size})
                0: case(saved_mem_addr2[1:0])
                        3:  memOut2_sliced = {{24{memOut_w0[31]}},memOut_w0[31:24]};      //lb     //endianess
                        2:  memOut2_sliced = {{24{memOut_w0[23]}},memOut_w0[23:16]};
                        1:  memOut2_sliced = {{24{memOut_w0[15]}},memOut_w0[15:8]};
                        0:  memOut2_sliced = {{24{memOut_w0[7]}},memOut_w0[7:0]};
                   endcase
                        
                1: case(saved_mem_addr2[1:0])
                        3: memOut2_sliced = {{16{memOut_w0[31]}},memOut_w0[31:24]};      //lh   //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {{16{memOut_w0[31]}},memOut_w0[31:16]};
                        1: memOut2_sliced = {{16{memOut_w0[23]}},memOut_w0[23:8]};
                        0: memOut2_sliced = {{16{memOut_w0[15]}},memOut_w0[15:0]};
                   endcase
                2: case(saved_mem_addr2[1:0])
                        1: memOut2_sliced = memOut_w0[31:8];   //spans two words, NOT YET SUPPORTED!
                        0: begin 
                            memOut2_sliced = memOut_w0;      //lw  
                            w1_out = memOut_w1;  
                           end 
                   endcase
                4: case(saved_mem_addr2[1:0])
                        3:  memOut2_sliced = {24'd0,memOut_w0[31:24]};      //lbu
                        2:  memOut2_sliced = {24'd0,memOut_w0[23:16]};
                        1:  memOut2_sliced = {24'd0,memOut_w0[15:8]};
                        0:  memOut2_sliced = {24'd0,memOut_w0[7:0]};
                   endcase
                5: case(saved_mem_addr2[1:0])
                        3: memOut2_sliced = {16'd0,memOut_w0};      //lhu //spans two words, NOT YET SUPPORTED!
                        2: memOut2_sliced = {16'd0,memOut_w0[31:16]};
                        1: memOut2_sliced = {16'd0,memOut_w0[23:8]};
                        0: memOut2_sliced = {16'd0,memOut_w0[15:0]};
                   endcase
            endcase
    end
 
    always_comb begin
        if(saved_mem_addr2 >= 32'h11000000)      
            MEM_DOUT2 = ioIn_buffer;  
        else 
            MEM_DOUT2 = memOut2_sliced;
            
            //sets 4 output words
            MEM_r0 = memOut_w0; 
            MEM_r1 = memOut_w1; 
            MEM_r2 = memOut_w2; 
            MEM_r3 = memOut_w3;   
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
