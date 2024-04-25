`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic br_lt,br_eq,br_ltu;
     
    instr_t de_ex_inst, de_inst;
    instr_t ex_mem_inst; 
    instr_t wb_inst;          
//==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc;
     logic [31:0] if_ir;
     
     always_ff @(posedge CLK) begin
                if_de_pc <= pc;
     end
     
     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
     PC PC_count  (.CLK(CLK), .RST(RESET), .PC_WRITE(pcWrite), .PC_SOURCE(pc_source),
        .JALR(jalr_pc), .JAL(jal_pc), .BRANCH(branch_pc), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(next_pc));
    
    logic [13:0] addr1;
    assign addr1 = pc[15:2];
     // reads only the instruction
     


//    always_ff @(posedge CLK) begin
//        IR <= if_ir;
//    end

     
//==== Instruction Decode ===========================================
    logic [31:0] de_opA, de_ex_opA;
    logic [31:0] de_opB, de_ex_opB;
    
    logic [31:0] de_ex_rs2, de_rs2; 
    logic [31:0] de_rs1, de_ex_rs1;

    
    
    
    opcode_t OPCODE;
    assign OPCODE_t = opcode_t'(opcode);
    
    assign de_inst.mem_type = IR[14:12];
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode=OPCODE;
   
    assign de_inst.rs1_used=    de_inst.rs1 != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;

    assign de_inst.rs2_used=    de_inst.rs1 != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL
                                && de_inst.opcode != OP_IMM; 
    logic ir30;
    assign ir30 = IR[30];
    logic [6:0] opcode;
    assign opcode = IR[6:0];
    logic [2:0] funct;
    assign funct = IR[14:12];                            
    
    BCG OTTER_BCG(.RS1(rs1), .RS2(de_ex_rs2), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
    
    // gather rs values, where do I send them? // 
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(wb_inst.regWrite), .ADR1(de_inst.rs2_addr), .ADR2(de_inst.rs1_addr), 
        .WA(wb_inst.rd_addr), .WD(wd), .RS1(de_rs1), .RS2(de_rs2));
    
    //DOUBLE CHECK BR VALUES LATER, MIGHT HAVE TO BE DELAYED FOR PIPELINE
    CU_DCDR OTTER_DCDR (.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), .BR_LT(br_lt),
     .BR_LTU(br_ltu), .ALU_FUN(de_inst.alu_fun), .ALU_SRCA(de_opA), .ALU_SRCB(de_opB), .PC_SOURCE(pc_sel),
     .REG_WRITE(de_inst.regWrite), .MEM_WE(de_inst.memWrite), .MEM_RDEN2(de_inst.memRead2),
      .RF_WR_SEL(de_inst.rf_wr_sel));
	
	//STILL NEED IMMEADIATE GENERATOR //
	
	always_ff @(posedge CLK) begin
        de_ex_opA <= de_opA;
        de_ex_opB <= de_opB;
        de_ex_inst <= de_inst;
        
        de_ex_rs1 <= de_rs1;
        de_ex_rs2 <= de_rs2;
        
     end
//==== Execute ======================================================
     
     
     logic [31:0] ex_mem_rs2;
     logic ex_mem_aluRes = 0;
     
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     
     //NEEDS ALUA AND ALUB SOURCE MUXES
     TwoMux OTTER_ALU_MUXA(.ALU_SRC_A(de_ex_opA), .RS1(de_ex_rs1), .U_TYPE(U_immed), .SRC_A(aluAin));
     FourMux OTTER_ALU_MUXB(.SEL(de_ex_opB), .ZERO(de_ex_rs2), .ONE(I_immed), 
                            .TWO(S_immed), .THREE(pc_out), .OUT(aluBin));
    
     
     // Creates a RISC-V ALU
     ALU OTTER_ALU (de_ex_inst.alu_fun, de_ex_opA, de_ex_opB, aluResult); // the ALU
     
     //BAG
     
     always_ff @(posedge CLK) begin
        ex_mem_rs2 <= de_ex_rs2;
        ex_mem_inst <= de_ex_inst;
        ex_mem_aluRes <= aluResult;
     end



//==== Memory ======================================================
      
     
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    
    //Memory File 
    // memRead1 is from IF block, hardwired high
    Memory OTTER_MEMORY(.MEM_CLK(CLK), .MEM_RDEN1(memRead1), .MEM_RDEN2('b0), 
        .MEM_WE2('b0), .MEM_ADDR1(addr1), .MEM_ADDR2(IOBUS_ADDR), .MEM_DIN2(IOBUS_OUT), .MEM_SIZE(size),
         .MEM_SIGN(sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(IR), .MEM_DOUT2(dout2));
 
    always_ff @(posedge CLK) begin
        wb_inst <= ex_mem_inst;
        
    end
 
 
     
//==== Write Back ==================================================
    
    
    FourMux OTTER_REG_MUX(.SEL(wb_inst.rf_wr_sel), 
            .ZERO(pc_out_inc), .ONE(32'b0), .TWO(dout2), .THREE(IOBUS_ADDR),
            .OUT(wd));


       
            
endmodule
