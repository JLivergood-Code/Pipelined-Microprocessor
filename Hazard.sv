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

`ifndef STRUCTS
    `define STRUCTS;
    typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
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
        logic memWrite;
        logic memRead2;
        logic regWrite;
        logic [1:0] rf_wr_sel;
        logic [2:0] mem_type;  //sign, size
        logic [31:0] pc;
    } instr_t;
`endif


module Hazard(
    input instr_t ex,
    input instr_t mem,
    input instr_t wb,
    
    output logic [1:0] FOR_MUX1_SEL,
    output logic [2:0] FOR_MUX2_SEL,
    output logic LW_STALL,
    
    output logic EX_FLUSH,
    output logic IF_FLUSH,
    output logic DEC_FLUSH
    );
    
    always_comb begin
        LW_STALL = 'b0;
        IF_FLUSH = 'b0;
        DEC_FLUSH = 'b0;
        EX_FLUSH = 'b0;
    
    //RS1 What does logic look like to stall for load word
    //RS1 DATA HAZARD CONTROL
        if(ex.rs1_used && (ex.rs1_addr == mem.rd_addr) && mem.regWrite) begin 
            if(mem.opcode == LOAD) begin
                LW_STALL = 'b1;
                EX_FLUSH = 'b1;
                
                //Is this necessary?
                FOR_MUX1_SEL = 2'b10;
                
            end else begin
                
                FOR_MUX1_SEL = 2'b01; 
            end
        end
            
        else if (ex.rs1_used && (ex.rs1_addr == wb.rd_addr) && wb.regWrite) begin FOR_MUX1_SEL = 2'b10; end
       
        else begin FOR_MUX1_SEL = 2'b00; end
        
        
        //RS2 DATA HAZARD CONTROL
        if(ex.rs2_used && (ex.rs2_addr == mem.rd_addr) && mem.regWrite) begin 
            if(mem.opcode == LOAD) begin
                LW_STALL = 'b1;
                EX_FLUSH = 'b1;
                
                //Is this necessary?
                FOR_MUX1_SEL = 2'b10;
                
            end else begin
                FOR_MUX2_SEL = 2'b01;
            end 
         end
            
        else if (ex.rs2_used && (ex.rs2_addr == wb.rd_addr) && wb.regWrite) begin FOR_MUX2_SEL = 2'b10; end
       
        else begin FOR_MUX2_SEL = 2'b00; end
    
    end
    
endmodule
