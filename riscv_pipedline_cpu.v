`timescale 1ns / 1ps


module adder #(parameter data_width = 32)(
    input [data_width-1:0] in1,
    input [data_width-1:0] in2,
    output wire [data_width-1:0] out
);
    assign out = in1 + in2;
endmodule

module ALU #(parameter data_width = 32, ctrlLen = 4)(
    input [data_width-1:0] in1,
    input [data_width-1:0] in2,
    input [ctrlLen-1:0] ALUctrl,
    output reg [data_width-1:0] ALUResult,
    output wire zero
);
    assign zero = (ALUResult == 0)? 1:0;
    always @(in1, in2, ALUctrl) begin
        case(ALUctrl)
            4'h0: ALUResult = in1 & in2; // and
            4'h1: ALUResult = in1 | in2; // or
            4'h2: ALUResult = in1 + in2; //add
            4'h3: ALUResult = in1 - in2; //sub
            4'h4: ALUResult = (in1 == in2); // bne
            4'h5: ALUResult = in1 << in2; // sll
            4'h6: ALUResult = in1 >> in2; // srl
            4'h7: ALUResult = ($signed(in1)) >>> in2; //sra
            4'h8: ALUResult = ($signed(in1) < $signed(in2))? 0:1; // blt
            4'h9: ALUResult = ($signed(in1) < $signed(in2))? 1:0; // bge
            default: ALUResult = 0;                
        endcase
    end
endmodule

module ALUcontrol #(parameter len = 4)(
    input [1:0] ALUOp,
    input [len-1:0] inst,
    output reg [len-1:0] ctrl
);
    always @(ALUOp, inst) begin
        ctrl <= 0;
        case(ALUOp)
            2'b00: ctrl <= 4'h2; // lw, sw, jal
            2'b01: begin
                case(inst[len-2:0])
                    3'b000: ctrl <= 4'h3; // beq
                    3'b001: ctrl <= 4'h4; // bne
                    3'b100: ctrl <= 4'h8; // blt
                    3'b101: ctrl <= 4'h9; // bge
                endcase
            end
            2'b10: begin
                case(inst[2:0])
                    3'b000: ctrl <= (inst[3])? 4'h3 : 4'h2; // sub, add
                    3'b111: ctrl <= 4'h0; // and
                    3'b110: ctrl <= 4'h1; // or
                    3'b001: ctrl <= 4'h5; // sll
                    3'b101: ctrl <= (inst[3])? 4'h7 : 4'h6; // sra, srl
                endcase
            end
            2'b11: begin
                case (inst[2:0])
                    3'b000: ctrl <= 4'h2; // addi
                    3'b111: ctrl <= 4'h0; // andi
                    3'b001: ctrl <= 4'h5; // slli
                    3'b101: ctrl <= (inst[3])? 4'h7 : 4'h6; // srai, srli
                endcase
            end
        endcase
    end
endmodule

module branch_compare #(parameter data_width = 32)(
    input [data_width-1:0] Rdata1,
    input [data_width-1:0] Rdata2,
    input Branch,
    input [2:0] funct3,
    output reg en
);
    initial begin en = 0; end
    always @(*) begin
        if (Branch == 0) begin en <= 0; end
        else begin
            case (funct3)
                3'b000: en <= (Rdata1 == Rdata2)? 1:0; // beq
                3'b001: en <= (Rdata1 == Rdata2)? 0:1; // bne
                3'b100: en <= (Rdata1 < Rdata2)? 1:0; // blt
                3'b101: en <= (Rdata1 < Rdata2)? 0:1; // bge
            endcase
        end
    end
endmodule

module branch_forward #(parameter addr_width = 8)(
    input Branch,
    input [addr_width-1:0] rs1,
    input [addr_width-1:0] rs2,
    input [addr_width-1:0] MEM_rd,
    input MEM_RegWrite,
    input MEM_MemRead,
    input [addr_width-1:0] EX_rd,
    input EX_RegWrite,
    output reg forward_1,
    output reg forward_2,
    output reg stall
);
    initial begin forward_1 = 0; forward_2 = 0; stall = 0; end
    always @(*) begin
        if (Branch == 1) begin
            forward_1 = ((MEM_rd == rs1) & MEM_RegWrite & ~MEM_MemRead);
            forward_2 = ((MEM_rd == rs2) & MEM_RegWrite & ~MEM_MemRead);
            stall = (MEM_RegWrite & MEM_MemRead & (MEM_rd == rs1 | MEM_rd == rs2)) | (EX_RegWrite & (EX_rd == rs1 | EX_rd == rs2));
        end 
        else begin {forward_1, forward_2, stall} = 3'b000; end
    end

endmodule

module Control(
    input [6:0] Ctrlin,
    input [2:0] funct3,
    output wire ALUSrc,
    output wire [1:0] ALUOp,
    output wire MemWrite,
    output wire MemRead,
    output wire RegWrite,
    output wire Branch,
    output wire MemtoReg,
    output wire WdataSrc,
    output wire RdataSrc,
    output wire storeType,
    output wire [1:0] loadType
);
    reg [12:0] bus = 0;
    assign {ALUSrc, ALUOp, MemWrite, MemRead, Branch, MemtoReg, RegWrite, WdataSrc, RdataSrc, storeType, loadType} = bus;
    always @(*) begin
        case(Ctrlin)
            7'b0110011: bus <= 13'b0100000100000; // add, sub, and, or, sll, srl
            7'b0010011: bus <= 13'b1110000100000; // addi, andi, slli
            7'b0000011: begin //lw
                case (funct3)
                    3'b000: bus <= 13'b1000101100001; //lb
                    3'b100: bus <= 13'b1000101100010; //lbu
                    default: bus <= 13'b1000101100000; //lw
                endcase
            end
            7'b0100011: begin //sw
                case (funct3)
                    3'b000: bus<= 13'b1001000000100; //sb
                    default: bus <= 13'b1001000000000; //sw
                endcase       
            end     
            7'b1100011: bus <= 13'b0010010000000; // beq, bne, bge, blt
            7'b1100111: bus <= 13'b0000010001000; // jalr
            7'b1101111: bus <= 13'b1000010110000; // jal
            default: bus <= 0;
        endcase
    end
endmodule

module DataMem #(parameter addr_width=32, data_width=32, memNum = 12)(
    input MemWrite,
    input MemRead,
    input [addr_width-1:0] addr,
    input [data_width-1:0] Wdata,
    input clk,
    input storeType,
    input [1:0] loadType,
    output reg [data_width-1:0] Rdata
);
    // To-Do: change
    localparam regNum = 1 << memNum;
    reg [7:0] dataMem [regNum-1:0];
    integer i;
    initial begin for(i=0;i<regNum;i=i+1) dataMem[i] = 0; end
    always @(negedge clk) begin
        if (MemRead == 0) Rdata <= 0;
        else begin
            case(loadType)
                2'b00: Rdata <= {dataMem[addr+3], dataMem[addr+2], dataMem[addr+1], dataMem[addr]}; // lw
                2'b01: Rdata <= {{24{dataMem[addr][7]}}, dataMem[addr]}; //lb
                2'b10: Rdata <= {24'b0, dataMem[addr]}; //lbu
                default: Rdata <= 0;                    
            endcase
        end
    end
    always @(posedge clk) begin
        if (MemWrite == 1'b1) begin 
            if (storeType == 1'b1) dataMem[addr] <= Wdata[7:0]; //sb
            else begin
                {dataMem[addr+3], dataMem[addr+2], dataMem[addr+1], dataMem[addr]} <= Wdata;
            end //sw
        end
    end
endmodule

module forward_mux #(parameter data_width = 32)(
    input [data_width-1:0] Rdata,
    input [data_width-1:0] MEM_Wdata,
    input [data_width-1:0] WB_Wdata,
    input [1:0] sel,
    output reg [data_width-1:0] out
);
    always @(*) begin
        case (sel)
            2'b10: out <= MEM_Wdata;
            2'b01: out <= WB_Wdata;
            default: out <= Rdata;
        endcase
    end
endmodule

module forward_unit #(parameter addr_width = 8)(
    input [addr_width-1:0] rs1,
    input [addr_width-1:0] rs2,
    input [addr_width-1:0] MEM_rd,
    input [addr_width-1:0] WB_rd,
    input MEM_RegWrite,
    input WB_RegWrite,
    output reg [1:0] forward_A,
    output reg [1:0] forward_B 
);
    initial begin forward_A = 0; forward_B = 0; end
    always @(*) begin
        if (MEM_RegWrite == 1'b1 & MEM_rd != 0 & MEM_rd == rs1) begin forward_A <= 2'b10; end
        else if (WB_RegWrite == 1'b1 & WB_rd != 0 & WB_rd == rs1) begin forward_A <= 2'b01; end
        else begin forward_A <= 0; end
    end
    always @(*) begin
        if (MEM_RegWrite == 1'b1 & MEM_rd != 0 & MEM_rd == rs2) begin forward_B <= 2'b10; end
        else if (WB_RegWrite == 1'b1 & WB_rd != 0 & WB_rd == rs2) begin forward_B <= 2'b01; end
        else begin forward_B <= 0; end
    end
endmodule

module hazard_unit #(parameter addr_width = 8)(
    input clk,
    input [addr_width-1:0] rs1,
    input [addr_width-1:0] rs2,
    input [addr_width-1:0] EX_rd,
    input EX_MemRead,
    output reg PCWrite,
    output reg IF_ID_Write,
    output reg hazard
);
    initial begin PCWrite = 1; IF_ID_Write = 1; hazard = 0; end
    always @(negedge clk) begin
        if (EX_MemRead == 1 & EX_rd != 0 & (rs1 == EX_rd | rs2 == EX_rd)) begin
            {PCWrite, IF_ID_Write, hazard} <= 3'b001;
        end
        else begin {PCWrite, IF_ID_Write, hazard} <= 3'b110; end
    end
endmodule

module ImmGen(
    input [31:0] M, 
    output reg [31:0] imm
);
    always @(M) begin
        if (M[6:0] == 7'h03 || M[6:0] == 7'h0f || M[6:0] == 7'h13 || M[6:0] == 7'h67 || M[6:0] == 7'h73) begin
            imm <= {{21{M[31]}}, M[30:20]}; // I-type
        end
        else if (M[6:0] == 7'h23) begin
            imm <= {{21{M[31]}}, M[30:25], M[11:7]}; // S-type
        end
        else if (M[6:0] == 7'h63) begin
            imm <= {{21{M[31]}}, M[7], M[30:25], M[11:8]}; // B-type
        end
        else if (M[6:0] == 7'h17 || M[6:0] == 7'h37) begin
            imm <= {{13{M[31]}}, M[30:12]}; // U-type
        end
        else if (M[6:0] == 7'h6f) begin
            imm <= {{13{M[31]}}, M[19:12], M[20], M[30:21]}; // J-type
        end
        else begin imm <= 0; end
    end
endmodule

module InstrMem #(parameter instrNum=8, addr_width=8, instr_width=32)(
    input [instr_width-1:0] addr,
    output reg [instr_width-1:0] instruction
);
    localparam regNum = 1 << instrNum;
    reg [instr_width-1:0] instrMem [regNum-1:0];
    integer i;
    initial begin 
        for (i=0;i<regNum;i=i+1) begin instrMem[i] = 0; end
        $readmemb("./test.txt", instrMem);
        instruction = 0;
    end
    always @(*) begin instruction = instrMem[addr >> 2]; end
endmodule

module SubInstr #(parameter addr_width=8, instr_width=32)(
    input [instr_width-1:0] instruction,
    output wire [addr_width-1:0] Raddr1,
    output wire [addr_width-1:0] Raddr2,
    output wire [addr_width-1:0] Waddr,
    output wire [6:0] opcode,
    output wire [3:0] ALUin,
    output wire [2:0] funct3
);
    assign opcode = instruction[6:0];
    assign ALUin = {instruction[30], instruction[14:12]};
    assign Raddr1 = instruction[19:15];
    assign Raddr2 = instruction[24:20];
    assign Waddr = instruction[11:7];
    assign funct3 = instruction[14:12];
endmodule

module mux_2_to_1 #(parameter data_width = 32)(
    input [data_width-1:0] in1,
    input [data_width-1:0] in2,
    input sel,
    output [data_width-1:0] out 
);
    assign out = (sel==0)? in1 : in2;
endmodule

module PC #(parameter addr_width = 32)(
    input [addr_width-1:0] nextPC,
    input clk,
    input en,
    output reg [addr_width-1:0] addr
);
    initial begin addr <= 0; end
    always @(posedge clk) begin if (en) addr <= nextPC; end
endmodule

module Register #(parameter addr_width = 8, data_width = 32)(
    input [addr_width-1:0] Raddr1,
    input [addr_width-1:0] Raddr2, 
    input [addr_width-1:0] Waddr,
    input [data_width-1:0] Wdata,
    input clk,
    input RegWrite,
    output reg [data_width-1:0] Rdata1,
    output reg [data_width-1:0] Rdata2
);
    localparam regNum = 1 << addr_width;
    reg [data_width-1:0] register [regNum-1:0];
    integer i;
    initial begin for(i=0;i<regNum;i=i+1) register[i] = 0; end

    always @(posedge clk) begin 
        if (RegWrite == 1'b1 & Waddr != 0) register[Waddr] <= Wdata; 
    end
    always @(negedge clk) begin
        Rdata1 = (RegWrite == 1 & Waddr == Raddr1)? Wdata:register[Raddr1]; 
        Rdata2 = (RegWrite == 1 & Waddr == Raddr2)? Wdata:register[Raddr2];
    end
endmodule

module shifter #(parameter data_width=32)(
    input [data_width-1:0] imm,
    output wire [data_width-1:0] out
);
    assign out = imm << 1;
endmodule

module state_register #(parameter length = 13)(
    input [length-1:0] in,
    input clk,
    input en,
    input clr,
    output reg [length-1:0] out
);
    initial begin out = 0; end
    always @(posedge clk) begin 
        if (en & ~clr) out <= in; 
        else out <= 0;
    end
endmodule

module top #(
    parameter data_width = 32,
    parameter addr_width = 8,
    parameter instr_width = 32,
    parameter len = 4
)(input clk);

    // IF:
    wire [instr_width-1:0] out1;
    wire [instr_width-1:0] out2;
    wire PCsrc;
    wire [instr_width-1:0] potentialPC;
    wire [instr_width-1:0] nextPC;
    wire [instr_width-1:0] addr;
    wire [instr_width-1:0] instruction;
    
    // ID:
    wire [instr_width-1:0] ID_addr;
    wire [instr_width-1:0] ID_instruction;
    wire [instr_width-1:0] ID_out1;
    wire [addr_width-1:0] Raddr1;
    wire [addr_width-1:0] Raddr2;
    wire [addr_width-1:0] Waddr;
    wire [6:0] opcode;
    wire [3:0] ALUin;
    wire [2:0] funct3;
    wire Branch;
    wire [instr_width-1:0] imm;
    wire [addr_width-1:0] WB_Waddr;
    wire MemRead;
    wire MemtoReg;
    wire [1:0] ALUOp;
    wire MemWrite;
    wire ALUSrc;
    wire RegWrite;
    wire WB_RegWrite;
    wire WdataSrc;
    wire RdataSrc;
    wire storeType;
    wire [1:0] loadType;
    wire [data_width-1:0] WdataReg;
    wire [data_width-1:0] Rdata1;
    wire [data_width-1:0] Rdata2;
    wire [data_width-1:0] rs1_data;
    wire [data_width-1:0] rs2_data;
    wire PCWrite;
    wire IF_ID_Write;
    wire hazard;
    wire [11:0] control_signal;
    wire [11:0] ID_EX_control;
    wire en;
    wire IF_ID_en;
    wire IF_ID_clr;
    wire forward_1;
    wire forward_2;
    wire [instr_width-1:0] temp;
    wire branch_stall;

    // EX:
    wire [instr_width-1:0] EX_imm;
    wire [instr_width-1:0] EX_out1;
    wire zero;
    wire [instr_width-1:0] EX_addr;
    wire [3:0] EX_ALUin;
    wire [addr_width-1:0] EX_Waddr;
    wire EX_MemRead;
    wire EX_MemtoReg;
    wire [1:0] EX_ALUOp;
    wire EX_MemWrite;
    wire EX_ALUSrc;
    wire EX_RegWrite;
    wire EX_WdataSrc;
    wire EX_RdataSrc;
    wire EX_storeType;
    wire [1:0] EX_loadType;
    wire [data_width-1:0] EX_Rdata1;
    wire [data_width-1:0] EX_Rdata2;
    wire [data_width-1:0] in2;
    wire [len-1:0] ctrl;
    wire [data_width-1:0] ALUResult;
    wire [1:0] forward_A;
    wire [1:0] forward_B;
    wire [data_width-1:0] ALU_in1;
    wire [data_width-1:0] ALU_in2;
    wire [addr_width-1:0] EX_Raddr1;
    wire [addr_width-1:0] EX_Raddr2;

    // MEM:
    wire [instr_width-1:0] MEM_out1;
    wire [addr_width-1:0] MEM_Waddr;
    wire MEM_MemRead;
    wire MEM_MemtoReg;
    wire MEM_MemWrite;
    wire MEM_RegWrite;
    wire MEM_WdataSrc;
    wire MEM_RdataSrc;
    wire MEM_storeType;
    wire [1:0] MEM_loadType;
    wire [data_width-1:0] MEM_Rdata1;
    wire [data_width-1:0] MEM_Rdata2;
    wire [data_width-1:0] MEM_ALUResult;
    wire [data_width-1:0] Rdata; // DataMem output
    wire [addr_width-1:0] MEM_Raddr2;
    wire MEMsrc;
    wire [data_width-1:0] MEM_in;

    // WB:
    wire [instr_width-1:0] WB_out1;
    wire WB_MemtoReg;
    wire WB_MemRead;
    wire WB_WdataSrc;
    wire WB_RdataSrc;
    wire [data_width-1:0] Wdata;
    wire [data_width-1:0] WB_ALUResult;
    wire [data_width-1:0] WB_Rdata;
    wire SelSrc;


    // IF:
    PC uut0(nextPC, clk, PCWrite, addr);
    InstrMem uut1(addr, instruction);
    adder uut2(addr, 4, out1);
    mux_2_to_1 uut3(out1, out2, PCsrc, potentialPC);
    mux_2_to_1 uut4(potentialPC, ALUResult, EX_RdataSrc, nextPC);

    // ID:
    assign IF_ID_en = (~PCsrc) & IF_ID_Write & (~branch_stall);
    assign IF_ID_clr = PCsrc & (~branch_stall);
    hazard_unit uut28(clk, Raddr1, Raddr2, EX_Waddr, EX_MemRead, PCWrite, IF_ID_Write, hazard);
    SubInstr uut5(ID_instruction, Raddr1, Raddr2, Waddr, opcode, ALUin, funct3);
    Control uut6(opcode, funct3, ALUSrc, ALUOp, MemWrite, MemRead, RegWrite, Branch, MemtoReg, WdataSrc, RdataSrc, storeType, loadType);
    ImmGen uut7(ID_instruction, imm);
    Register register_file(Raddr1, Raddr2, WB_Waddr, WdataReg, clk, WB_RegWrite, rs1_data, rs2_data);
    assign control_signal = {ALUOp, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, WdataSrc, RdataSrc, storeType, loadType};
    mux_2_to_1 #(12) uut29(control_signal, 12'b0, hazard, ID_EX_control);
    branch_compare uut31(Rdata1, Rdata2, Branch, funct3, en);
    assign PCsrc = en | WdataSrc;
    branch_forward uut32(Branch, Raddr1, Raddr2, MEM_Waddr, MEM_RegWrite, MEM_MemRead, EX_Waddr, EX_RegWrite, forward_1, forward_2, branch_stall);
    mux_2_to_1 #(32) uut33(rs1_data, MEM_ALUResult, forward_1, Rdata1);
    mux_2_to_1 #(32) uut34(rs2_data, MEM_ALUResult, forward_2, Rdata2);
    shifter uut12(imm, temp);
    adder uut13(ID_addr, temp, out2);
    
    // EX:
    forward_unit uut25(EX_Raddr1, EX_Raddr2, MEM_Waddr, WB_Waddr, MEM_RegWrite, WB_RegWrite, forward_A, forward_B);
    forward_mux uut26(EX_Rdata1, MEM_ALUResult, Wdata, forward_A, ALU_in1);
    forward_mux uut27(EX_Rdata2, MEM_ALUResult, Wdata, forward_B, in2);
    mux_2_to_1 uut8(in2, EX_imm, EX_ALUSrc, ALU_in2); //
    ALUcontrol uut9(EX_ALUOp, EX_ALUin, ctrl);
    ALU uut11(ALU_in1, ALU_in2, ctrl, ALUResult, zero);


    // MEM
    DataMem uut14(MEM_MemWrite, MEM_MemRead, MEM_ALUResult, MEM_in, clk, MEM_storeType, MEM_loadType, Rdata);
    assign MEMsrc = (WB_Waddr != 0) & (WB_Waddr == MEM_Raddr2) & WB_MemRead & MEM_MemWrite;
    mux_2_to_1 uut30(MEM_Rdata2, Wdata, MEMsrc, MEM_in);

    // WB
    mux_2_to_1 uut15(WB_ALUResult, WB_Rdata, WB_MemtoReg, Wdata);
    mux_2_to_1 uut16(Wdata, WB_out1, SelSrc, WdataReg);
    assign SelSrc = WB_WdataSrc | WB_RdataSrc;

    // state register
    // IF/ID:
    state_register #(96) uut17({addr, instruction, out1}, clk, IF_ID_en, IF_ID_clr, {ID_addr, ID_instruction, ID_out1});
    // ID/EX:
    state_register #(96) uut18({imm, ID_addr, ID_out1}, clk, 1'b1, 1'b0, {EX_imm, EX_addr, EX_out1});
    state_register #(92) uut19({Waddr, ALUin, Rdata1, Rdata2, Raddr1, Raddr2}, clk, 1'b1, 1'b0, {EX_Waddr, EX_ALUin, EX_Rdata1, EX_Rdata2, EX_Raddr1, EX_Raddr2});
    state_register #(12) uut20(ID_EX_control, clk, 1'b1, 1'b0, {EX_ALUOp, EX_MemRead, EX_MemtoReg, EX_MemWrite, EX_ALUSrc, EX_RegWrite, EX_WdataSrc, EX_RdataSrc, EX_storeType, EX_loadType});
    // EX/MEM:
    state_register #(144) uut21({EX_out1, EX_Waddr, EX_Rdata1, in2, ALUResult, EX_Raddr2}, clk, 1'b1, 1'b0, {MEM_out1, MEM_Waddr, MEM_Rdata1, MEM_Rdata2, MEM_ALUResult, MEM_Raddr2});
    state_register #(4) uut22({EX_MemRead, EX_MemtoReg, EX_MemWrite, EX_RegWrite}, clk, 1'b1, 1'b0, {MEM_MemRead, MEM_MemtoReg, MEM_MemWrite, MEM_RegWrite});
    state_register #(5) uut23({EX_loadType, EX_storeType, EX_WdataSrc, EX_RdataSrc}, clk, 1'b1, 1'b0, {MEM_loadType, MEM_storeType,MEM_WdataSrc, MEM_RdataSrc});
    // MEM/WB:
    state_register #(109) uut24({MEM_out1, MEM_MemtoReg, MEM_WdataSrc, MEM_ALUResult, Rdata, MEM_RegWrite, MEM_Waddr, MEM_RdataSrc, MEM_MemRead}, clk, 1'b1, 1'b0, {WB_out1, WB_MemtoReg, WB_WdataSrc, WB_ALUResult, WB_Rdata, WB_RegWrite, WB_Waddr, WB_RdataSrc, WB_MemRead});
endmodule //top