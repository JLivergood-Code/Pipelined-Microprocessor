`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 06/04/2024 12:18:44 PM
// Design Name: 
// Module Name: MEM_FSM
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


module MEM_FSM(
    
    );
parameter SA_WAYS = 4;
parameter NUM_BLOCKS = 16;
parameter BLOCK_SIZE = 8;
parameter INDEX_SIZE = 4;
parameter WORD_OFFSET_SIZE = 3;
parameter BYTE_OFFSET = 0;
parameter TAG_SIZE = 32 - INDEX_SIZE - WORD_OFFSET_SIZE - BYTE_OFFSET;
endmodule
