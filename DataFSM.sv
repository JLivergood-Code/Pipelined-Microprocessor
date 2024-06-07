`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/23/2024 02:08:34 PM
// Design Name: 
// Module Name: DataFSM
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


module DataFSM(input hit, input miss, input CLK, input RST, input read, input write, input dirty, output logic Load, output logic writeback, output logic FSM_Write,
               output logic FSM_Read);

    typedef enum{
        START,
        READ,
        LOAD,
        WRITE_BACK,
        WRITE,
        MEM_WRITE
    } state_type;
    
state_type PS, NS;

always_ff @(posedge CLK) begin
    PS <= NS;
end

always_comb begin
    update = 1'b1;
    pc_stall = 0;
    case (PS)
        START: begin
            update = 1'b0;
            if(hit) begin
                NS = ST_READ_CACHE;
            end
            else if(miss) begin
                pc_stall = 1'b1;
                NS = ST_READ_MEM;
            end
            else NS = ST_READ_CACHE;
        end
    endcase
end

endmodule
