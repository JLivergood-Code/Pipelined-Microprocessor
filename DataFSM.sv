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


module DataFSM(input hit, input miss, input CLK, input RST, output logic update, output logic pc_stall);

    parameter PS;
    parameter NS;

always_ff @(posedge CLK) begin
    PS <= NS;
end

always_comb begin
    update = 1'b1;
    pc_stall = 0;
    case (PS)
        ST_READ_CACHE: begin
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
        ST_READ_MEM: begin
            pc_stall = 1'b1;
            NS = ST_READ_CACHE;
        end
        default: NS = ST_READ_CACHE;
    endcase
end

endmodule
