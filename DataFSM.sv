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


module DataFSM(input hit, input miss, input CLK, input RST, input read, input write, input dirty, output logic load, output logic writeback, output logic FSM_Write,
               output logic FSM_Read, output logic stall);

    typedef enum{
        START,
        LOAD,
        WRITE_BACK,
        MEM_WRITE
    } state_type;
    
state_type PS, NS;

always_ff @(posedge CLK) begin
    if(RST == 1)
        PS <= START;
    else
        PS <= NS;
end

always_comb begin
    load = 1'b1;
    stall = 0;
    case (PS)
        START: begin
            load = 1'b0;
            if(read) begin
                if(hit) begin
                    NS = START;
                end
                else if(miss && dirty) begin
                    NS = WRITE_BACK;
                    stall = 1'b1;
                end
                else if(miss && ~dirty)begin 
                    NS = LOAD;
                    stall = 1'b1;
                end
                else NS = START;
            end
            else if(write) begin
                if(hit) begin
                    NS = START;
                end
                else if(miss) begin
                    NS = MEM_WRITE;
                    stall = 1'b1;
                end
            end
            else begin
                NS = START;
            end
      end
      
      
    endcase
end

endmodule
