`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/07/2024 01:54:02 PM
// Design Name: 
// Module Name: Hazard
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

module Hazard(
    input instr_t ex,
    input instr_t mem,
    input instr_t wb,
    
    output [1:0] FOR_MUX1_SEL,
    output [2:0] FOR_MUX2_SEL,
    output LW_STALL
    );
    
    //RS1 What does logic look like to stall for load word
    //RS1 DATA HAZARD CONTROL
    if(ex.rs1_used && ex.rs1_addr == mem.rd_addr && mem.regWrite) begin assign FOR_MUX1_SEL = 2'b01; end
        
    else if (ex.rs1_used && ex.rs1_addr == wb.rd_addr && wb.regWrite) begin assign FOR_MUX1_SEL = 2'b10; end
   
    else begin assign FOR_MUX1_SEL = 2'b00; end
    
    
    //RS2 DATA HAZARD CONTROL
    if(ex.rs2_used && ex.rs2_addr == mem.rd_addr && mem.regWrite) begin assign FOR_MUX2_SEL = 2'b01; end
        
    else if (ex.rs2_used && ex.rs2_addr == wb.rd_addr && wb.regWrite) begin assign FOR_MUX2_SEL = 2'b10; end
   
    else begin assign FOR_MUX2_SEL = 2'b00; end
    
    
    
endmodule
