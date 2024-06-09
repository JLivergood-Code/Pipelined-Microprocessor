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

`ifndef STRUCTS
    `define STRUCTS;
     typedef enum logic [6:0] {
           FLUSH    = 7'b0000000,
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
        logic [31:0] ir;
        logic [31:0] pc;
    } instr_t;
`endif

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, jal_pc, branch_pc, int_pc,A,B,
        I_immed,S_immed,U_immed, B_immed, J_immed,
        aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    logic [31:0] IR, if_IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [2:0] pc_sel, pc_source;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic [31:0] wd;
    
    //hazard signals
    logic stall, if_flush, de_flush, ex_flush;
    logic [1:0] for_mux2_sel;
    logic [1:0] for_mux1_sel;
    
    logic br_lt,br_eq,br_ltu;
    logic CACHE_stall = 0;
     
    instr_t de_ex_inst, de_inst;
    instr_t ex_mem_inst; 
    instr_t wb_inst;          
//==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc;
     logic [31:0] if_de_IR;
//     logic [31:0] if_IR;
     
     logic prev_stall;
     logic [2:0] buff_pc_source;
     
     
     assign pcWrite = ~stall & ~CACHE_stall; 	//Hardwired high, assuming now hazards
     assign memRead1 = ~stall; 	//Fetch new instruction every cycle
     assign pc_source = 3'b0;
     
     PC PC_count  (.CLK(CLK), .RST(RESET), .PC_WRITE(pcWrite), .PC_SOURCE(pc_source),
        .JALR(jalr_pc), .JAL(jal_pc), .BRANCH(branch_pc), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(next_pc), .STALL(stall));
    
    logic [31:0] addr1;
    assign addr1 = pc;
     // reads only the instruction
     
     logic [31:0] w0, w1, w2, w3, w4, w5, w6, w7;
     logic update, hit, miss;
     
     //IR Cache Modules
     IR_MEM IR_MEM (.a(addr1), .w0(w0), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .w5(w5), .w6(w6), .w7(w7));
     Cache IR_CACHE (.PC(addr1), .CLK(CLK), .update(update), 
        .w0(w0), .w1(w1), .w2(w2), .w3(w3), .w4(w4), .w5(w5), .w6(w6), .w7(w7),
        .rd(if_IR), .hit(hit), .miss(miss));
     
     CacheFSM IR_FSM (.hit(hit), .miss(miss), .CLK(CLK), .RST(RESET), .update(update), .pc_stall(stall));
     
     
     
     TwoMux FLUSH_MUX (.A(if_IR), .B(32'b0), .SEL(de_flush), .OUT(if_de_IR));

        
//always_ff @(posedge CLK) begin
//          if (~stall) begin if_de_pc <= pc; end
//     end

    always_ff @(posedge CLK) begin
        if_flush <= 'b0;
        
        if_de_pc <= pc-4;
        if(stall || CACHE_stall) begin  
            IR <= IR;
//            buff_pc_source <= buff_pc_source;
//            prev_pc_out <= pc_out;
            if (de_flush) begin
                if_flush <= de_flush;
            end
        end
        else begin 
            IR <= if_de_IR;
//            if_flush <= 'b0;
//            buff_pc_source <= pc_source; 
        end
        if(de_flush || if_flush) begin IR <= 32'b0; end
        else begin 
            IR <= if_de_IR;
//            buff_pc_source <= pc_source; 
        end
    end

     
//==== Instruction Decode ===========================================
    logic de_opA, de_ex_opA;
    logic [1:0] de_opB, de_ex_opB;
    
    logic [31:0] de_ex_rs2, de_rs2; 
    logic [31:0] de_rs1, de_ex_rs1;
    logic [31:0] flush_values;

    logic [31:0] ex_J_immed, ex_I_immed, ex_U_immed, ex_B_immed, ex_S_immed;
    logic [2:0] ex_pc_sel;
   
    logic buff_ex_flush;
    logic [1:0] buff_for_mux1, buff_for_mux2;
    
    opcode_t OPCODE;
    assign opcode = IR[6:0];
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst.mem_type = IR[14:12];
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode = OPCODE;
    assign de_inst.pc = if_de_pc+4;
    assign de_inst.ir = IR;
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;

    assign de_inst.rs2_used=    de_inst.rs2_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL
                                && de_inst.opcode != JALR
                                && de_inst.opcode != OP_IMM; 
    logic ir30;
    assign ir30 = IR[30];
    
    logic [2:0] funct, ex_funct;
    assign funct = IR[14:12];                            
    
    
    //HOW DO I FORWARD CORRECTLY FOR THIS
//    FourMux For_BCG_MUX1 (.ONE(aluResult), .TWO(), .THREE(), .FOUR(), .SEL(), .OUT());
//    FourMux FOR_BCG_MUX2(.ONE(aluResult), .TWO(), .THREE(), .FOUR(), .SEL(), .OUT());
    
    
//    BCG OTTER_BCG(.RS1(de_rs1), .RS2(de_rs2), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
    
        //Branch Addres Generator
//     BAG OTTER_BAG(.RS1(de_rs1), .I_TYPE(I_immed), .J_TYPE(J_immed), .B_TYPE(B_immed), .FROM_PC(de_inst.pc),
//         .JAL(jal_pc), .JALR(jalr_pc), .BRANCH(branch_pc));

    
    // gather rs values, where do I send them? // 
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(wb_inst.regWrite), .ADR1(de_inst.rs1_addr), .ADR2(de_inst.rs2_addr), 
        .WA(wb_inst.rd_addr), .WD(wd), .RS1(de_rs1), .RS2(de_rs2));
    
    //DOUBLE CHECK BR VALUES LATER, MIGHT HAVE TO BE DELAYED FOR PIPELINE
    CU_DCDR OTTER_DCDR (.IR_30(ir30), .IR_OPCODE(de_inst.opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), .BR_LT(br_lt),
     .BR_LTU(br_ltu), .ALU_FUN(de_inst.alu_fun), .ALU_SRCA(de_opA), .ALU_SRCB(de_opB), .PC_SOURCE(pc_sel),
     .REG_WRITE(de_inst.regWrite), .MEM_WE(de_inst.memWrite), .MEM_RDEN2(de_inst.memRead2),
      .RF_WR_SEL(de_inst.rf_wr_sel));
	
	//STILL NEED IMMEADIATE GENERATOR //
	ImmediateGenerator OTTER_IMGEN(.IR(IR[31:7]), .U_TYPE(U_immed), .I_TYPE(I_immed), .S_TYPE(S_immed),
        .B_TYPE(B_immed), .J_TYPE(J_immed));
    
    
    
    assign pc_source = pc_sel;    
        
	always_ff @(posedge CLK) begin
        if(~CACHE_stall) begin    
            ex_J_immed <= J_immed;
            ex_I_immed <= I_immed;
            ex_U_immed <= U_immed;
            ex_B_immed <= B_immed;
            ex_S_immed <= S_immed;
            
            de_ex_inst <= de_inst;
            ex_funct <= funct;
            
            buff_for_mux1 <= for_mux1_sel;
            buff_for_mux2 <= for_mux2_sel;
            
            if(ex_flush) begin 
                //FLUSH EX Instruction
                de_ex_rs1 <= 32'b0;
                de_ex_rs2 <= 32'b0;
                
                de_ex_inst.opcode <= FLUSH;
                de_ex_inst.memWrite <= 'b0;
                de_ex_inst.regWrite <= 'b0;
                de_ex_inst.memRead2 <= 'b0;
             
                ex_pc_sel <= 'b0;
            end else begin
            
                buff_ex_flush <= ex_flush;
                de_ex_opA <= de_opA;
                de_ex_opB <= de_opB;
                de_ex_inst <= de_inst;
                
                de_ex_rs1 <= de_rs1;
                de_ex_rs2 <= de_rs2;
                
               
             
                ex_pc_sel <= pc_sel;
    
            end
         end
        
     end
//==== Execute ======================================================
     
     
     logic [31:0] ex_mem_rs2;
     logic [31:0] ex_mem_aluRes;
     
     logic [31:0] aluA_forwarded;
     logic [31:0] aluB_forwarded;
     
//     assign pc_source = ex_pc_sel;
     
     //NEEDS ALUA AND ALUB SOURCE MUXES
     TwoMuxALU OTTER_ALU_MUXA(.ALU_SRC_A(de_ex_opA), .RS1(aluA_forwarded), .U_TYPE(ex_U_immed), .SRC_A(aluAin));
     FourMux OTTER_ALU_MUXB(.SEL(de_ex_opB), .ZERO(aluB_forwarded), .ONE(ex_I_immed), 
                            .TWO(ex_S_immed), .THREE(de_ex_inst.pc), .OUT(aluBin));
    
    //Forwarding Muxes
    FourMux ForwardMux1 (.SEL(buff_for_mux1), .ZERO(de_ex_rs1), .ONE(ex_mem_aluRes), .TWO(wd), .OUT(aluA_forwarded));
    FourMux ForwardMux2 (.SEL(buff_for_mux2), .ZERO(de_ex_rs2), .ONE(ex_mem_aluRes), .TWO(wd), .OUT(aluB_forwarded));
    
    
     
     // Creates a RISC-V ALU
     ALU OTTER_ALU(.SRC_A(aluAin), .SRC_B(aluBin), .ALU_FUN(de_ex_inst.alu_fun), .RESULT(aluResult));
     
     //Branch COndition Generator
     BCG OTTER_BCG(.RS1(aluA_forwarded), .RS2(aluB_forwarded), 
            .OPCODE(de_ex_inst.opcode), .IR_FUNCT(ex_funct), 
            .PC_SOURCE(pc_source));

     //BAG
     //Branch Addres Generator
     BAG OTTER_BAG(.RS1(aluA_forwarded), .I_TYPE(ex_I_immed), .J_TYPE(ex_J_immed), .B_TYPE(ex_B_immed), .FROM_PC(de_ex_inst.pc),
         .JAL(jal_pc), .JALR(jalr_pc), .BRANCH(branch_pc));
     
     
     always_ff @(posedge CLK) begin
//        if(buff_ex_flush) begin
//            ex_mem_inst.opcode <= FLUSH;
//            ex_mem_inst.memWrite <= 'b0;
//            ex_mem_inst.regWrite <= 'b0;
//            ex_mem_inst.memRead2 <= 'b0;
            
////            ex_pc_sel = 'b0;
//        end else begin 
        if(~CACHE_stall) begin
            ex_mem_rs2 <= aluB_forwarded;
            ex_mem_inst <= de_ex_inst;
            ex_mem_aluRes <= aluResult;
        end
     end



//==== Memory ======================================================
    logic [31:0] wb_dout2;  
    logic [31:0] wb_IOBUS_ADDR;
    logic [31:0] dout2;
     
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    
    //Memory File 
    // memRead1 is from IF block, hardwired high
//    OTTER_mem_byte OTTER_MEMORY(.MEM_CLK(CLK), .MEM_READ1('b0), .MEM_READ2(ex_mem_inst.memRead2), 
//        .MEM_WRITE2(ex_mem_inst.memWrite), .MEM_ADDR2(IOBUS_ADDR), .MEM_DIN2(IOBUS_OUT), .MEM_SIZE(ex_mem_inst.mem_type[1:0]),
//         .MEM_SIGN(ex_mem_inst.mem_type[2]), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR),  .MEM_DOUT2(dout2));


    //MEM Cache
    MEM_Cache MEM_Cache (.addr(ex_mem_aluRes), .in_data(ex_mem_rs2), .CLK(CLK), .RST(RESET), 
        .MEM_SIZE(ex_mem_inst.mem_type[1:0]), .MEM_SIGN(ex_mem_inst.mem_type[2]), .IOBUS_IN(IOBUS_IN), .IOBUS_WR(IOBUS_WR),
        .MEM_WRITE(ex_mem_inst.memWrite), .MEM_READ(ex_mem_inst.memRead2), 
        .DOUT2(dout2), .CACHE_stall(CACHE_stall));
 
    //Removed Modules for Cache
    //.MEM_ADDR1(addr1)
    //.MEM_READ1(memRead1),
    //.MEM_DOUT1(if_IR),
 
    always_ff @(posedge CLK) begin
        wb_inst <= ex_mem_inst;
        wb_dout2 <= dout2;
        wb_IOBUS_ADDR <= IOBUS_ADDR;
    end
    
//    assign wb_inst = ex_mem_inst;
//    assign wb_dout2 = dout2;
//    assign wb_IOBUS_ADDR = IOBUS_ADDR;
 
 
     
//==== Write Back ==================================================
    
    
    FourMux OTTER_REG_MUX(.SEL(wb_inst.rf_wr_sel), 
            .ZERO(wb_inst.pc+4), .ONE(32'b0), .TWO(dout2), .THREE(wb_IOBUS_ADDR),
            .OUT(wd));

//====== HAZARD =====================================================

// TODO: Hazard Detection Does not seem to be working
//       Mux forwarding needs to be created
//       Need to do all of control hazard

    Hazard Hazard_Module (.dec(de_inst), .ex(de_ex_inst), .mem(ex_mem_inst), .wb(eb_inst), .pc_source(pc_source),
         .LW_STALL(stall), .FOR_MUX1_SEL(for_mux1_sel), .FOR_MUX2_SEL(for_mux2_sel),
        /*.IF_FLUSH(if_flush),*/ .DEC_FLUSH(de_flush), .EX_FLUSH(ex_flush));
       
            
endmodule
