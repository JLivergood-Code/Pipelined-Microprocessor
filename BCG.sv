`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly San Luis Obispo
// Engineer: Diego Curiel
// Create Date: 02/09/2023 11:30:51 AM
// Module Name: BCG
//////////////////////////////////////////////////////////////////////////////////

module BCG(
    input [31:0] RS1,
    input [31:0] RS2,
    input [6:0] OPCODE,
    input [2:0] IR_FUNCT,
    output logic [2:0] PC_SOURCE
    //output logic BR_EQ, BR_LT, BR_LTU
    );
    
    logic BR_EQ, BR_LT, BR_LTU;
    
    //Assign outputs using conditional logic operators.
    assign BR_LT = $signed(RS1) < $signed(RS2);
    assign BR_LTU = RS1 < RS2;
    assign BR_EQ = RS1 == RS2;
    
    
    always_comb begin
        PC_SOURCE = 3'b000;
        if(OPCODE == 7'b1100011) begin //if branch
            case(IR_FUNCT)
                3'b000: begin //BEQ
                    if (BR_EQ == 1'b1)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000; 
                end
                3'b001: begin //BNE
                    if (BR_EQ == 1'b0)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000; 
                end
                3'b100: begin //blt
                    if (BR_LT == 1'b1)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000;
                end
                3'b101: begin //bge
                    if (BR_LT == 1'b0)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000;
                end
                3'b110: begin //bltu
                    if (BR_LTU == 1'b1)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000;
                end
                3'b111: begin //bgeu
                    if (BR_LTU == 1'b0)
                        PC_SOURCE = 3'b010;
                    else
                        PC_SOURCE = 3'b000;
                end
          endcase
        end
        if(OPCODE == 7'b1101111) begin // JAL
                PC_SOURCE = 3'b011;
            end
        if(OPCODE == 7'b1100111) begin // JALR
                PC_SOURCE = 3'b001;
         end
     end
endmodule
