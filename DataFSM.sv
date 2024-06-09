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


module MEM_FSM(
    input hit, 
    input miss, 
    input CLK, 
    input RST, 
    input read, 
    input write, 
    input dirty, 
    output logic load, 
    output logic writeback, 
    output logic FSM_Write,
    output logic FSM_Read,
    output logic set_dirty, 
    output logic stall,
    output logic [1:0] counter);

    typedef enum{
        START,
        LOAD,
        WRITE_BACK,
        MEM_WRITE,
        MEM_READ,
        PROCESS
    } state_type;
    
state_type PS, NS;
logic [1:0] count_next;


always_ff @(posedge CLK) begin
    if(RST == 1) begin
        PS <= START;
        counter <= 0;
    end
    else begin
        PS <= NS;
        counter <= count_next;
        
    end
end

always_comb begin
    load = 1'b0;
    stall = 0;
    FSM_Write = 1'b0;
    FSM_Read = 1'b0;
    writeback = 1'b0;
    count_next = counter;
//    set_dirty = 'b0;
    case (PS)
        START: begin
            load = 1'b0;
            if(read) begin
                if(hit) begin
                    NS = PROCESS;
                end
                else if(miss && dirty) begin
                    NS = WRITE_BACK;
                    stall = 1'b1;
                end
                else if(miss && ~dirty)begin 
                    NS = MEM_READ;
                    stall = 1'b1;
                end
                else NS = START;
            end
            else if(write) begin
                if(hit) begin
                    set_dirty = 'b1;
                    NS = PROCESS;
                end
                else if(miss) begin
                    set_dirty = 'b0;
                    NS = MEM_WRITE;
                    stall = 1'b1;
                end
            end
            else begin
                NS = START;
            end
      end

      PROCESS: begin
            if(write) begin
                if(hit) begin
                    set_dirty = 'b0;
                end
             end
             NS = START;
      end

      WRITE_BACK: begin
        count_next = counter + 'b1;
        stall = 1'b1;
        writeback = 1'b1;
        FSM_Write = 1'b1;
        if(counter == 3) begin
          NS = MEM_READ;
          count_next = 0;
        end        
        else begin
          NS = WRITE_BACK;
        end
      end

      MEM_WRITE: begin
        NS = MEM_READ;
        FSM_Write = 1'b1;
        stall = 1'b1;
      end
      
      MEM_READ: begin
        load = 1'b1;
        FSM_Read = 1'b1;
        stall = 1'b1;
        NS = LOAD;
      end

      LOAD: begin
        set_dirty = 'b0;
        load = 1'b1;
        FSM_Read = 1'b0;
        NS = PROCESS;
        stall = 1'b1;
      end

      default: NS = START;
      
    endcase
end

endmodule
