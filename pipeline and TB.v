module InstructionMemory (
 input [31:0] address,
 output [31:0] instruction
);

 // ROM to store instructions
 reg [31:0] memory [0:255]; // Can be expanded as needed

 // Fetch instruction based on the address (word-aligned)
 assign instruction = memory[address[9:2]]; // Ignoring last 2 bits (word addressing)

endmodule


module DataMemory (
 input clk,
 input MemRead,
 input MemWrite,
 input [31:0] address,
 input [31:0] writeData,
 output [31:0] readData
);

 // 256 × 32‑bit RAM
 reg [31:0] memory [0:255];

 // Synchronous write
 always @(posedge clk) begin
 if (MemWrite) begin
 memory[address[9:2]] <= writeData;
 end
 end

 // Combinational read via wire
 assign readData = (MemRead)
 ? memory[address[9:2]]
 : 32'b0;

endmodule



// 3. Program Counter (PC)
module ProgramCounter (
 input wire clk,
 input wire reset,
 input wire stall, // Prevent PC update on stall
 input wire [31:0] nextPC, // Next PC value (PC+4 or branch/jump target)
 output reg [31:0] PC // Current PC value
);

 always @(posedge clk or posedge reset) begin
 if (reset)
 PC <= 32'b0; // Initialize PC to 0 on reset
 else if (!stall)
 PC <= nextPC; // Update PC if no stall
 // else: stall → keep PC unchanged
 end

endmodule



module PipelineRegister_IF_ID (
 input clk,
 input reset,
 input IF_Flush, // Clear register (e.g., on branch taken)
 input IF_Stall, // Prevent update (e.g., on hazard)
 input [31:0] pc_in,
 input [31:0] instruction_in,
 output reg [31:0] pc_out,
 output reg [31:0] instruction_out
);

 always @(posedge clk or posedge reset) begin
 if (reset || IF_Flush) begin
 pc_out <= 32'b0;
 instruction_out <= 32'b0;
 end else if (!IF_Stall) begin
 pc_out <= pc_in;
 instruction_out <= instruction_in;
 end
 // else: stall active → hold current values
 end

endmodule


module PipelineRegister_ID_EX (
 input clk,
 input reset,
 input ID_Flush, // Clear the contents (e.g., branch taken)
 input ID_Stall, // Hold the current values (e.g., hazard stall)

 // Control Signals
 input RegDst_in,
 input ALUSrc_in,
 input [1:0] ALUOp_in,
 input MemRead_in,
 input MemWrite_in,
 input RegWrite_in,
 input MemToReg_in,

 // Data Signals
 input [31:0] pc_in,
 input [31:0] regData1_in,
 input [31:0] regData2_in,
 input [31:0] imm_in,
 input [4:0] rs_in,
 input [4:0] rt_in,
 input [4:0] rd_in,
 input Branch_in,

 // Outputs
 output reg RegDst_out,
 output reg ALUSrc_out,
 output reg [1:0] ALUOp_out,
 output reg MemRead_out,
 output reg MemWrite_out,
 output reg RegWrite_out,
 output reg MemToReg_out,

 output reg [31:0] pc_out,
 output reg [31:0] regData1_out,
 output reg [31:0] regData2_out,
 output reg [31:0] imm_out,
 output reg [4:0] rs_out,
 output reg [4:0] rt_out,
 output reg [4:0] rd_out,
 output reg Branch_out
);

 always @(posedge clk or posedge reset) begin
 if (reset || ID_Flush) begin
 // Clear control + data signals (NOP effect)
 RegDst_out <= 0;
 ALUSrc_out <= 0;
 ALUOp_out <= 2'b00;
 MemRead_out <= 0;
 MemWrite_out <= 0;
 RegWrite_out <= 0;
 MemToReg_out <= 0;
 Branch_out <= 1'b0;
 pc_out <= 0;
 regData1_out <= 0;
 regData2_out <= 0;
 imm_out <= 0;
 rs_out <= 0;
 rt_out <= 0;
 rd_out <= 0;
 end else if (!ID_Stall) begin
 // Normal update
 RegDst_out <= RegDst_in;
 ALUSrc_out <= ALUSrc_in;
 ALUOp_out <= ALUOp_in;
 MemRead_out <= MemRead_in;
 MemWrite_out <= MemWrite_in;
 RegWrite_out <= RegWrite_in;
 MemToReg_out <= MemToReg_in;
 Branch_out <= Branch_in;
 pc_out <= pc_in;
 regData1_out <= regData1_in;
 regData2_out <= regData2_in;
 imm_out <= imm_in;
 rs_out <= rs_in;
 rt_out <= rt_in;
 rd_out <= rd_in;
 end
 // else: ID_Stall == 1 → hold old values (do nothing)
 end

endmodule

module PipelineRegister_EX_MEM (
 input clk,
 input reset,

 // Control Signals
 input MemRead_in,
 input MemWrite_in,
 input RegWrite_in,
 input MemToReg_in,

 // Data
 input [31:0] aluResult_in,
 input [31:0] regData2_in,
 input [4:0] writeReg_in,

 // Outputs
 output reg MemRead_out,
 output reg MemWrite_out,
 output reg RegWrite_out,
 output reg MemToReg_out,

 output reg [31:0] aluResult_out,
 output reg [31:0] regData2_out,
 output reg [4:0] writeReg_out
);

 always @(posedge clk or posedge reset) begin
 if (reset) begin
 {MemRead_out, MemWrite_out, RegWrite_out, MemToReg_out} <= 0;
 aluResult_out <= 0;
 regData2_out <= 0;
 writeReg_out <= 0;
 end else begin
 MemRead_out <= MemRead_in;
 MemWrite_out <= MemWrite_in;
 RegWrite_out <= RegWrite_in;
 MemToReg_out <= MemToReg_in;

 aluResult_out <= aluResult_in;
 regData2_out <= regData2_in;
 writeReg_out <= writeReg_in;
 end
 end

endmodule


module PipelineRegister_MEM_WB (
 input clk,
 input reset,

 // Control
 input RegWrite_in,
 input MemToReg_in,

 // Data
 input [31:0] readData_in,
 input [31:0] aluResult_in,
 input [4:0] writeReg_in,

 // Outputs
 output reg RegWrite_out,
 output reg MemToReg_out,
 output reg [31:0] readData_out,
 output reg [31:0] aluResult_out,
 output reg [4:0] writeReg_out
);

 always @(posedge clk or posedge reset) begin
 if (reset) begin
 RegWrite_out <= 0;
 MemToReg_out <= 0;
 readData_out <= 0;
 aluResult_out <= 0;
 writeReg_out <= 0;
 end else begin
 RegWrite_out <= RegWrite_in;
 MemToReg_out <= MemToReg_in;
 readData_out <= readData_in;
 aluResult_out <= aluResult_in;
 writeReg_out <= writeReg_in;
 end
 end

endmodule






module RegisterFile (
 input Clk,
 input RegWrite,
 input [4:0] readReg1,
 input [4:0] readReg2,
 input [4:0] writeReg,
 input [31:0] writeData,
 output reg [31:0] readData1,
 output reg [31:0] readData2
);

 reg [31:0] Registers [0:31];
 integer i;
 initial begin
 for (i = 0; i < 32; i = i + 1)
 Registers[i] = 32'b0;
 end

 always @(*) begin
 readData1 = (readReg1 == 0) ? 32'b0 : Registers[readReg1];
 readData2 = (readReg2 == 0) ? 32'b0 : Registers[readReg2];
 end

 always @(posedge Clk) begin
 if (RegWrite && (writeReg != 0))
 Registers[writeReg] <= writeData;
 end

endmodule


// Main Control Unit
module MainControl (
 input [5:0] opcode, // bits [31:26] of the instruction
 output RegDst, // 1 = rd, 0 = rt
 output ALUSrc, // 1 = immediate, 0 = register
 output MemToReg, // 1 = data mem, 0 = ALU result
 output RegWrite, // write register file
 output MemRead, // read data memory
 output MemWrite, // write data memory
 output Branch, // beq
 output Jump, // j
 output [1:0] ALUOp // passes to ALUControl
);

 // Default all control signals to 0
 // (will override in the case statement below)
 reg r_RegDst,
 r_ALUSrc,
 r_MemToReg,
 r_RegWrite,
 r_MemRead,
 r_MemWrite,
 r_Branch,
 r_Jump;
 reg [1:0] r_ALUOp;

 assign RegDst = r_RegDst;
 assign ALUSrc = r_ALUSrc;
 assign MemToReg = r_MemToReg;
 assign RegWrite = r_RegWrite;
 assign MemRead = r_MemRead;
 assign MemWrite = r_MemWrite;
 assign Branch = r_Branch;
 assign Jump = r_Jump;
 assign ALUOp = r_ALUOp;

 always @(*) begin
 // defaults
 r_RegDst = 1'b0;
 r_ALUSrc = 1'b0;
 r_MemToReg = 1'b0;
 r_RegWrite = 1'b0;
 r_MemRead = 1'b0;
 r_MemWrite = 1'b0;
 r_Branch = 1'b0;
 r_Jump = 1'b0;
 r_ALUOp = 2'b00;

 case (opcode)
 6'b000000: begin // R-type (add, sub, and, or, slt, ...)
 r_RegDst = 1'b1;
 r_ALUSrc = 1'b0;
 r_MemToReg = 1'b0;
 r_RegWrite = 1'b1;
 r_MemRead = 1'b0;
 r_MemWrite = 1'b0;
 r_Branch = 1'b0;
 r_Jump = 1'b0;
 r_ALUOp = 2'b10;
 end

 6'b100011: begin // lw
 r_RegDst = 1'b0;
 r_ALUSrc = 1'b1;
 r_MemToReg = 1'b1;
 r_RegWrite = 1'b1;
 r_MemRead = 1'b1;
 r_MemWrite = 1'b0;
 r_Branch = 1'b0;
 r_Jump = 1'b0;
 r_ALUOp = 2'b00;
 end

 6'b101011: begin // sw
 r_RegDst = 1'bx; // don't care
 r_ALUSrc = 1'b1;
 r_MemToReg = 1'bx; // don't care
 r_RegWrite = 1'b0;
 r_MemRead = 1'b0;
 r_MemWrite = 1'b1;
 r_Branch = 1'b0;
 r_Jump = 1'b0;
 r_ALUOp = 2'b00;
 end

 6'b000100: begin // beq
 r_RegDst = 1'bx;
 r_ALUSrc = 1'b0;
 r_MemToReg = 1'bx;
 r_RegWrite = 1'b0;
 r_MemRead = 1'b0;
 r_MemWrite = 1'b0;
 r_Branch = 1'b1;
 r_Jump = 1'b0;
 r_ALUOp = 2'b01;
 end

 6'b000010: begin // j
 r_RegDst = 1'bx;
 r_ALUSrc = 1'bx;
 r_MemToReg = 1'bx;
 r_RegWrite = 1'b0;
 r_MemRead = 1'b0;
 r_MemWrite = 1'b0;
 r_Branch = 1'b0;
 r_Jump = 1'b1;
 r_ALUOp = 2'b00;
 end
 
 endcase
 end

endmodule

module ALU (
 input wire [31:0] input1, // First ALU operand
 input wire [31:0] input2, // Second ALU operand
 input wire [3:0] ALUControl, // ALU operation code
 output reg [31:0] result, // ALU result
 output wire zero // 1 if result == 0 (for branch)
);

 // Combinational result logic
 always @(*) begin
 case (ALUControl)
 4'b0000: result = input1 & input2; // AND
 4'b0001: result = input1 | input2; // OR
 4'b0010: result = input1 + input2; // ADD
 4'b0110: result = input1 - input2; // SUB
 4'b0111: result = ($signed(input1) < $signed(input2)) ? 32'd1 : 32'd0; // SLT
 default: result = 32'd0; // NOP/undefined
 endcase
 end

 // Zero flag for branch decisions
 assign zero = (result == 32'd0);

endmodule

module ALUControl (
 input [1:0] ALUOp, // From main control: 00=lw/sw/addi, 01=beq, 10=R‑type
 input [5:0] funct, // funct field from the instruction (for R‑type)
 output reg [3:0] ALUControl // To drive the ALU’s operation select
);

 always @(*) begin
 case (ALUOp)
 2'b00: // load/store/addi — always ADD
 ALUControl = 4'b0010;
 2'b01: // beq — SUBTRACT to check for zero
 ALUControl = 4'b0110;
 2'b10: // R‑type: decode funct field
 case (funct)
 6'b100000: ALUControl = 4'b0010; // ADD
 6'b100010: ALUControl = 4'b0110; // SUB
 6'b100100: ALUControl = 4'b0000; // AND
 6'b100101: ALUControl = 4'b0001; // OR
 6'b101010: ALUControl = 4'b0111; // SLT
 default: ALUControl = 4'b0000; // default safe
 endcase
 default:
 ALUControl = 4'b0000; // safe default
 endcase
 end

endmodule


// Forwarding Unit
// Resolves data hazards by selecting ALU operands from later pipeline stages
module ForwardingUnit (
 input wire EX_MEM_RegWrite, // RegWrite from EX/MEM stage
 input wire [4:0] EX_MEM_RegRd, // destination reg# in EX/MEM
 input wire MEM_WB_RegWrite, // RegWrite from MEM/WB stage
 input wire [4:0] MEM_WB_RegRd, // destination reg# in MEM/WB
 input wire [4:0] ID_EX_RegRs, // source rs in ID/EX
 input wire [4:0] ID_EX_RegRt, // source rt in ID/EX
 output reg [1:0] ForwardA, // ALU input A select
 output reg [1:0] ForwardB // ALU input B select
);

 always @(*) begin
 // Default: take operands from ID/EX register file outputs
 ForwardA = 2'b00;
 ForwardB = 2'b00;

 // ---------- Source A forwarding ----------
 // If EX/MEM will write to a register and it's the same as ID/EX.rs
 if (EX_MEM_RegWrite && (EX_MEM_RegRd != 0) && (EX_MEM_RegRd == ID_EX_RegRs)) begin
 ForwardA = 2'b10; // take ALU input A from EX/MEM stage
 end
 // Else, if MEM/WB will write and matches ID/EX.rs
 else if (MEM_WB_RegWrite && (MEM_WB_RegRd != 0) && (MEM_WB_RegRd == ID_EX_RegRs)) begin
 ForwardA = 2'b01; // take ALU input A from MEM/WB stage
 end

 // ---------- Source B forwarding ----------
 if (EX_MEM_RegWrite && (EX_MEM_RegRd != 0) && (EX_MEM_RegRd == ID_EX_RegRt)) begin
 ForwardB = 2'b10; // take ALU input B from EX/MEM stage
 end
 else if (MEM_WB_RegWrite && (MEM_WB_RegRd != 0) && (MEM_WB_RegRd == ID_EX_RegRt)) begin
 ForwardB = 2'b01; // take ALU input B from MEM/WB stage
 end
 end

endmodule


// Hazard Detection Unit
// Detects load‑use hazards and generates stall/flush controls
module HazardDetectionUnit (
 input wire ID_EX_MemRead, // MemRead signal from ID/EX stage
 input wire [4:0] ID_EX_RegisterRt, // Rt field from ID/EX (load destination)
 input wire [4:0] IF_ID_RegisterRs, // Rs field from IF/ID (next instr)
 input wire [4:0] IF_ID_RegisterRt, // Rt field from IF/ID (next instr)
 output reg PCWrite, // 1 = allow PC update; 0 = stall PC
 output reg IF_ID_Write, // 1 = allow IF/ID update; 0 = stall IF/ID
 output reg control_mux // 1 = force EX control signals to zero (bubble)
);

 always @(*) begin
 // Default: no hazard — normal operation
 PCWrite = 1'b1;
 IF_ID_Write = 1'b1;
 control_mux = 1'b0;

 // Load‑use hazard: ID/EX is loading, and next instr uses that register
 if (ID_EX_MemRead &&
 ((ID_EX_RegisterRt == IF_ID_RegisterRs) ||
 (ID_EX_RegisterRt == IF_ID_RegisterRt))) begin
 // Stall the pipeline by:
 PCWrite = 1'b0; // freeze PC
 IF_ID_Write = 1'b0; // freeze IF/ID
 control_mux = 1'b1; // insert bubble in EX by zeroing controls
 end
 end

endmodule


// 10. Flush Unit (for Branch)
module FlushUnit (
 input wire Branch, // Control signal from Decode stage
 input wire Zero, // Result of comparison from ALU
 output reg flush // Output flush signal for IF/ID
);

 always @(*) begin
 if (Branch && Zero)
 flush = 1'b1; // Flush if beq is taken
 else
 flush = 1'b0;
 end

endmodule

// Top-level pipelined MIPS CPU datapath
module PipelineCPU(
 input wire clk,
 input wire reset
);

 //---- IF stage wires ----
 wire [31:0] PC, nextPC, PC_plus4, branch_target, jump_target;
 wire [31:0] instruction;
 wire PCWrite, IF_ID_Write, control_mux, IF_Flush;
 wire IF_Stall = ~IF_ID_Write;
 // PC source select: branch taken or jump
 
 wire ID_EX_Branch;

 //---- IF stage ----
 ProgramCounter pc_reg(
 .clk(clk),
 .reset(reset),
 .stall(~PCWrite),
 .nextPC(nextPC),
 .PC(PC)
 );

 InstructionMemory imem(
 .address(PC),
 .instruction(instruction)
 );

 // IF/ID pipeline register
 wire [31:0] IF_ID_PC, IF_ID_Instruction;
 wire [31:0] readData1, readData2;

 PipelineRegister_IF_ID if_id(
 .clk(clk),
 .reset(reset),
 .IF_Flush(IF_Flush),
 .IF_Stall(IF_Stall),
 .pc_in(PC),
 .instruction_in(instruction),
 .pc_out(IF_ID_PC),
 .instruction_out(IF_ID_Instruction)
 );

 //---- ID stage wires ----
 wire [5:0] opcode = IF_ID_Instruction[31:26];
 wire [4:0] rs = IF_ID_Instruction[25:21];
 wire [4:0] rt = IF_ID_Instruction[20:16];
 wire [4:0] rd = IF_ID_Instruction[15:11];
 wire [15:0] imm16 = IF_ID_Instruction[15:0];
 wire [25:0] jaddr = IF_ID_Instruction[25:0];

 // Control signals
 wire RegDst, ALUSrc, MemToReg, RegWrite;
 wire MemRead, MemWrite, Branch, Jump;
 wire [1:0] ALUOp;
 MainControl ctrl(
 .opcode(opcode),
 .RegDst(RegDst),
 .ALUSrc(ALUSrc),
 .MemToReg(MemToReg),
 .RegWrite(RegWrite),
 .MemRead(MemRead),
 .MemWrite(MemWrite),
 .Branch(Branch),
 .Jump(Jump),
 .ALUOp(ALUOp)
 );

 // Sign-extend and shift for branch offset
 wire [31:0] imm_ext = {{16{imm16[15]}}, imm16};
 

 // PC+4, branch and jump target computation
 assign PC_plus4 = PC + 32'd4;
 wire [31:0] ID_PC_plus4   = IF_ID_PC + 32'd4;
wire [31:0] imm_shift     = imm_ext << 2;
assign branch_target      = ID_PC_plus4 + imm_shift;

 assign jump_target = {IF_ID_PC[31:28], jaddr, 2'b00};

 //---- Hazard detection & flush ----
 // Stall on load-use hazard
 wire ID_EX_MemRead;
 wire [4:0] ID_EX_Rs, ID_EX_Rt;
 assign ID_EX_MemRead = MemRead_ex;

 HazardDetectionUnit hazard(
 .ID_EX_MemRead (ID_EX_MemRead),
 .ID_EX_RegisterRt(ID_EX_Rt),
 .IF_ID_RegisterRs(rs),
 .IF_ID_RegisterRt(rt),
 .PCWrite(PCWrite),
 .IF_ID_Write(IF_ID_Write),
 .control_mux(control_mux)
 );
 // Flush on taken branch
 wire zero;
 FlushUnit flushu(
 .Branch(ID_EX_Branch),
 .Zero(zero),
 .flush(IF_Flush)
 );

 //---- Register File ----
 // Write-back stage signals
 wire WB_RegWrite, WB_MemToReg;
 wire [31:0] WB_readData, WB_aluResult;
 wire [4:0] WB_writeReg;
 wire [31:0] writeBackData = WB_MemToReg ? WB_readData : WB_aluResult;

 RegisterFile regfile(
 .Clk(clk),
 .RegWrite(WB_RegWrite),
 .readReg1(rs),
 .readReg2(rt),
 .writeReg(WB_writeReg),
 .writeData(writeBackData),
 .readData1(readData1),
 .readData2(readData2)
 );

 //---- ID/EX pipeline register ----
 wire [31:0] ID_EX_PC, ID_EX_regData1, ID_EX_regData2, ID_EX_imm;
 wire RegDst_ex, ALUSrc_ex, RegWrite_ex;
 wire MemRead_ex, MemWrite_ex, MemToReg_ex;
 wire [1:0] ALUOp_ex;
 wire [4:0] ID_EX_writeRegSel;

 PipelineRegister_ID_EX id_ex(
 .clk (clk),
 .reset (reset),
 .ID_Flush (IF_Flush),
 .ID_Stall (1'b0),

 // Control signals now include Branch

 .RegDst_in (control_mux ? 1'b0 : RegDst),
 .ALUSrc_in (control_mux ? 1'b0 : ALUSrc),
 .ALUOp_in (control_mux ? 2'b00 : ALUOp),
 .MemRead_in (control_mux ? 1'b0 : MemRead),
 .MemWrite_in (control_mux ? 1'b0 : MemWrite),
 .RegWrite_in (control_mux ? 1'b0 : RegWrite),
 .MemToReg_in (control_mux ? 1'b0 : MemToReg),
 .Branch_in (control_mux ? 1'b0 : Branch),

 // Data signals
 .pc_in (IF_ID_PC),
 .regData1_in (readData1),
 .regData2_in (readData2),
 .imm_in (imm_ext),
 .rs_in (rs),
 .rt_in (rt),
 .rd_in (rd),

 // Control outputs → your new wires

 .RegDst_out (RegDst_ex),
 .ALUSrc_out (ALUSrc_ex),
 .ALUOp_out (ALUOp_ex),
 .MemRead_out (MemRead_ex),
 .MemWrite_out (MemWrite_ex),
 .RegWrite_out (RegWrite_ex),
 .MemToReg_out (MemToReg_ex),
 .Branch_out (ID_EX_Branch),

 // Data outputs → your new wires
 .pc_out (ID_EX_PC),
 .regData1_out (ID_EX_regData1),
 .regData2_out (ID_EX_regData2),
 .imm_out (ID_EX_imm),

 .rs_out (ID_EX_Rs),
 .rt_out (ID_EX_Rt),
 .rd_out (ID_EX_writeRegSel)
 );

 //---- EX stage wires ----
 wire [1:0] ForwardA, ForwardB;
 wire [31:0] A_mux1, B_mux1, ALU_input2;
 wire [3:0] ALUControlSig;
 wire [31:0] ALU_result;
 wire [4:0] EX_writeReg;

 // Forwarding
 ForwardingUnit fwd(
 .EX_MEM_RegWrite(EX_MEM_RegWrite),
 .EX_MEM_RegRd (EX_MEM_writeReg),
 .MEM_WB_RegWrite(WB_RegWrite),
 .MEM_WB_RegRd (WB_writeReg),
 .ID_EX_RegRs (ID_EX_Rs),
 .ID_EX_RegRt (ID_EX_Rt),
 .ForwardA (ForwardA),
 .ForwardB (ForwardB)
 );

 // ALUControl uses low 6 bits of immediate (instruction[5:0]) as funct
 ALUControl aluctrl(
 .ALUOp(ALUOp_ex),
 .funct(ID_EX_imm[5:0]),
 .ALUControl(ALUControlSig)
 );

 // MUX A
 assign A_mux1 = (ForwardA == 2'b10) ? EX_MEM_aluResult :
 (ForwardA == 2'b01) ? writeBackData :
 ID_EX_regData1;
 // MUX B pre-ALUSrc
 assign B_mux1 = (ForwardB == 2'b10) ? EX_MEM_aluResult :
 (ForwardB == 2'b01) ? writeBackData :
 ID_EX_regData2;
 // ALU second input: reg or immediate
 assign ALU_input2 = ALUSrc_ex ? ID_EX_imm : B_mux1;

 ALU alu(
 .input1 (A_mux1),
 .input2 (ALU_input2),
 .ALUControl (ALUControlSig),
 .result (ALU_result),
 .zero (zero)
 );

 // Choose write register: rd for R-type, rt for I-type
 assign EX_writeReg = RegDst_ex ? ID_EX_writeRegSel : ID_EX_Rt;

 //---- EX/MEM pipeline register ----
 wire EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_MemToReg;
 wire [31:0] EX_MEM_aluResult, EX_MEM_regData2;
 wire [4:0] EX_MEM_writeReg;

 PipelineRegister_EX_MEM ex_mem(
 .clk(clk),
 .reset(reset),
 .MemRead_in (MemRead_ex),
 .MemWrite_in(MemWrite_ex),
 .RegWrite_in(RegWrite_ex),
 .MemToReg_in(MemToReg_ex),
 .aluResult_in(ALU_result),
 .regData2_in(B_mux1),
 .writeReg_in(EX_writeReg),
 .MemRead_out (EX_MEM_MemRead),
 .MemWrite_out(EX_MEM_MemWrite),
 .RegWrite_out(EX_MEM_RegWrite),
 .MemToReg_out(EX_MEM_MemToReg),
 .aluResult_out(EX_MEM_aluResult),
 .regData2_out(EX_MEM_regData2),
 .writeReg_out(EX_MEM_writeReg)
 );

 //---- MEM stage ----
 wire [31:0] MEM_readData;
 DataMemory dmem(
 .clk(clk),
 .MemRead(EX_MEM_MemRead),
 .MemWrite(EX_MEM_MemWrite),
 .address(EX_MEM_aluResult),
 .writeData(EX_MEM_regData2),
 .readData(MEM_readData)
 );

 //---- MEM/WB pipeline register ----
 PipelineRegister_MEM_WB mem_wb(
 .clk(clk),
 .reset(reset),
 .RegWrite_in(EX_MEM_RegWrite),
 .MemToReg_in(EX_MEM_MemToReg),
 .readData_in(MEM_readData),
 .aluResult_in(EX_MEM_aluResult),
 .writeReg_in(EX_MEM_writeReg),
 .RegWrite_out(WB_RegWrite),
 .MemToReg_out(WB_MemToReg),
 .readData_out(WB_readData),
 .aluResult_out(WB_aluResult),
 .writeReg_out(WB_writeReg)
 );

 //---- Next PC MUX ----
 // after your PipelineRegister_ID_EX:
wire PCSrc= ID_EX_Branch & zero;

// then your final mux:
assign nextPC = Jump
              ? jump_target
              : PCSrc
                ? branch_target
                : PC_plus4;

endmodule

`timescale 1ns/1ps

module datapath_tb;

// Clock & reset
reg clk;
reg reset;

// Instantiate your top‐level CPU
PipelineCPU uut (
.clk (clk),
.reset (reset)
);

// Generate a 10 ns period clock
initial begin
clk = 0;
forever #5 clk = ~clk;
end

//============================================================================
// 1) Preload instruction memory (IMEM)
//============================================================================
initial begin
// R-type
uut.imem.memory[0] = 32'h00221820; // ADD $3, $1, $2 => $3 = 10 + 5 = 15
uut.imem.memory[1] = 32'h00222022; // SUB $4, $1, $2 => $4 = 10 - 5 = 5
uut.imem.memory[2] = 32'h00223024; // AND $6, $1, $2 => $6 = 10 & 5 = 0
uut.imem.memory[3] = 32'h00223825; // OR $7, $1, $2 => $7 = 10 | 5 = 15
uut.imem.memory[4] = 32'h0022402A; // SLT $8, $1, $2 => $8 = (10<5)?1:0=0

// I-type
uut.imem.memory[5] = 32'h8C2A0008; // LW $10, 8($1) => $10 = Mem[ (10+8)/4 = 4 ]
uut.imem.memory[6] = 32'hAC2B000C; // SW $11,12($1) => Mem[ (10+12)/4 = 5 ] = $11

// BEQ (not taken)
uut.imem.memory[7] = 32'h10220001; // BEQ $1,$2,+1 → not taken, falls through
uut.imem.memory[8] = 32'h00000000; // NOP

// JUMP
uut.imem.memory[9] = 32'h0800000C; // J 0x00000030
uut.imem.memory[10] = 32'h00000000; // NOP

// Padding
uut.imem.memory[11] = 32'h00000000;
uut.imem.memory[12] = 32'h00000000;
end

//============================================================================
// 2) Preload register file
//============================================================================
initial begin
// regfile instance is named 'regfile' and its array is 'Registers'
uut.regfile.Registers[1] = 32'd10;
uut.regfile.Registers[2] = 32'd5;
uut.regfile.Registers[11] = 32'hABCDEF12;
end

//============================================================================
// 3) Preload data memory
//============================================================================
initial begin
// dmem instance is named 'dmem' and its array is 'memory'
uut.dmem.memory[4] = 32'h12345678;
end

//============================================================================
// 4) Apply reset, run, then check results
//============================================================================
initial begin
// Assert reset for 2 cycles
reset = 1;
#20;
reset = 0;

// Let it run ~20 more cycles to flush the pipeline
#200;

$display("\n===== FINAL REGISTER & MEMORY CHECKS =====");

// ADD
if (uut.regfile.Registers[3] === 32'd15)
$display("ADD Test: SUCCESS ($3 = %h)", uut.regfile.Registers[3]);
else
$display("ADD Test: ERROR ($3 = %h, expected 0F)", uut.regfile.Registers[3]);

// SUB
if (uut.regfile.Registers[4] === 32'd5)
$display("SUB Test: SUCCESS ($4 = %h)", uut.regfile.Registers[4]);
else
$display("SUB Test: ERROR ($4 = %h, expected 05)", uut.regfile.Registers[4]);

// AND
if (uut.regfile.Registers[6] === (10 & 5))
$display("AND Test: SUCCESS ($6 = %h)", uut.regfile.Registers[6]);
else
$display("AND Test: ERROR ($6 = %h, expected 00)", uut.regfile.Registers[6]);

// OR
if (uut.regfile.Registers[7] === (10 | 5))
$display("OR Test: SUCCESS ($7 = %h)", uut.regfile.Registers[7]);
else
$display("OR Test: ERROR ($7 = %h, expected 0F)", uut.regfile.Registers[7]);

// SLT
if (uut.regfile.Registers[8] === 32'd0)
$display("SLT Test: SUCCESS ($8 = %h)", uut.regfile.Registers[8]);
else
$display("SLT Test: ERROR ($8 = %h, expected 00)", uut.regfile.Registers[8]);

 
if (uut.regfile.Registers[10] === 32'h12345678)
$display("LW Test: SUCCESS ($10 = %h)", uut.regfile.Registers[10]);
else
$display("LW Test: ERROR ($10 = %h, expected 12345678)", uut.regfile.Registers[10]);

// SW
if (uut.dmem.memory[5] === 32'hABCDEF12)
$display("SW Test: SUCCESS (Mem[5] = %h)", uut.dmem.memory[5]);
else
$display("SW Test: ERROR (Mem[5] = %h, expected ABCDEF12)", uut.dmem.memory[5]);

// BEQ (should fall through → PC >= 0x2C before jump)
if (uut.PC >= 32'h0000002C)
$display("BEQ Test: SUCCESS (PC = %h)", uut.PC);
else
$display("BEQ Test: ERROR (PC = %h, expected >= 2C)", uut.PC);

// JUMP (should land at or beyond 0x30)
if (uut.PC >= 32'h00000030)
$display("JUMP Test: SUCCESS (PC = %h)", uut.PC);
else
$display("JUMP Test: ERROR (PC = %h, expected >= 30)", uut.PC);

$display("\nALL TESTS COMPLETE\n");
$finish;
end

//============================================================================
// 5) Optional waveform / debug print
//============================================================================
always @(posedge clk) begin
 
$display("T=%0t | IF: PC=%h, Instr=%h", $time, uut.PC, uut.instruction);
$display(" ID: Instr=%h", uut.if_id.instruction_out);
$display(" EX: ALUin1=%h, ALUin2=%h", uut.id_ex.regData1_out, uut.id_ex.regData2_out);
 
end

endmodule

// `timescale 1ns/1ps
// module datapath_tb;

//   // Clock & reset
//   reg clk;
//   reg reset;

//   // Flags for our checks
//   reg forwarded_ex, forwarded_mem;
//   reg stalled;
//   reg flushed;

//   // Instantiate the pipelined CPU
//   PipelineCPU uut (
//     .clk   (clk),
//     .reset (reset)
//   );

//   // 10 ns clock
//   initial begin
//     clk = 0;
//     forever #5 clk = ~clk;
//   end

//   //============================================================================
//   // 1) Preload IMEM with three little tests back-to-back
//   //============================================================================
//   initial begin
//     // --- A) Forwarding test ---
//     // sub $2,$1,$3
//     uut.imem.memory[0] = 32'h00231022;
//     // and $12,$2,$5   ← needs EX→ID forwarding
//     uut.imem.memory[1] = 32'h00456024;
//     // or $13,$6,$2    ← needs MEM→ID forwarding
//     uut.imem.memory[2] = 32'h00C26825;

//     // --- B) Load-use stall test ---
//     // lw  $3,20($4)
//     uut.imem.memory[3] = 32'h8C830014;
//     // and $7,$3,$1    ← should cause a one-cycle stall
//     uut.imem.memory[4] = 32'h00613824;

//     // --- C) Taken-branch flush test ---
//     // BEQ $1,$1,+1    ← always taken
//     uut.imem.memory[5] = 32'h10210001;
//     // OR  $7,$1,$2    ← should be squashed
//     uut.imem.memory[6] = 32'h00223825;
//     // AND $8,$1,$2    ← should execute
//     uut.imem.memory[7] = 32'h00224024;


//   end

//   //============================================================================
//   // 2) Initial register file contents
//   //============================================================================
//   initial begin
//     uut.regfile.Registers[1] = 32'd10;
//     uut.regfile.Registers[3] = 32'd2;
//     uut.regfile.Registers[5] = 32'd3;
//     uut.regfile.Registers[6] = 32'd1;
//     // R4 stays 0 (base for LW)
//   end

//   //============================================================================
//   // 3) Data memory preload for the LW
//   //    LW $3,20($4) → addr = 20/4 = index 5
//   //============================================================================
//   initial begin
//     uut.dmem.memory[5] = 32'hDEAD_BEEF;
//   end

//   //============================================================================
//   // 4) Apply reset, run, then check PASS/FAIL
//   //============================================================================
//   initial begin
//     forwarded_ex  = 0;
//     forwarded_mem = 0;
//     stalled       = 0;
//     flushed       = 0;

//     // hold reset for 2 cycles
//     reset = 1;
//     #20;
//     reset = 0;

//     // run long enough to propagate everything
//     #200;

//     $display("\n===== PIPELINE CONTROL CHECKS =====");
//     if (forwarded_ex && forwarded_mem)
//       $display("FORWARDING Test: PASS");
//     else
//       $display("FORWARDING Test: FAIL (EX=%b, MEM=%b)",
//               forwarded_ex, forwarded_mem);

//     if (stalled)
//       $display("STALL Test: PASS");
//     else
//       $display("STALL Test: FAIL");

//     if (flushed)
//       $display("FLUSH Test: PASS");
//     else
//       $display("FLUSH Test: FAIL");

//     $display("\nALL TESTS COMPLETE\n");
//     $finish;
//   end

//   //============================================================================
//   // 5) Monitors (one per event), each uses a zero-time #0 delay
//   //============================================================================

//       // --- EX/MEM → EX forwarding for the AND $12,$2,$5 test ---
//   // when rs=2, rt=5 reach ID/EX, we expect ForwardA == 2’b10
//   always @(posedge clk) begin
//     if (uut.id_ex.rs_out  === 5'd2 &&
//         uut.id_ex.rt_out  === 5'd5) begin
//       #0
//       forwarded_ex = (uut.fwd.ForwardA === 2'b10);
//     end
//   end

//   // --- MEM/WB → EX forwarding for the OR $13,$6,$2 test ---
//   // when rs=6, rt=2 reach ID/EX, we expect ForwardB == 2’b01
//   always @(posedge clk) begin
//     if (uut.id_ex.rs_out  === 5'd6 &&
//         uut.id_ex.rt_out  === 5'd2) begin
//       #0
//       forwarded_mem = (uut.fwd.ForwardB === 2'b01);
//     end
//   end



//   // B) Stall: when AND $7,$3,$1 (0x00613824) is in IF/ID *and* control_mux==1
//   //    then PCWrite=0 & IF_ID_Write=0 on that same cycle
//   always @(posedge clk) begin
//     if (uut.if_id.instruction_out === 32'h00613824
//         && uut.hazard.control_mux) begin
//       #0 stalled = (uut.hazard.PCWrite === 1'b0
//                   && uut.hazard.IF_ID_Write === 1'b0);
//     end
//   end

//   // C) Flush: when BEQ $1,$1,+1 (0x10210001) is in IF/ID
//   //    flushu.flush must be 1 on that cycle
//   // C) Flush: when the flush line actually asserts
// always @(posedge clk) begin
//   #0;  // delta‐delay to avoid races
//   if (uut.flushu.flush)  
//     flushed = 1;
// end


//   //============================================================================
//   // 6) Debug print (no delay)
//   //============================================================================
//   always @(posedge clk) begin
//     $display(
//       "T=%0t | IF_ID_Instr=%h | FwdA=%b | FwdB=%b | PCW=%b,IFIDW=%b,cmx=%b | flush=%b",
//       $time,
//       uut.if_id.instruction_out,
//       uut.fwd.ForwardA,
//       uut.fwd.ForwardB,
//       uut.hazard.PCWrite, uut.hazard.IF_ID_Write, uut.hazard.control_mux,
//       uut.flushu.flush
     
//     );
//   end

// endmodule
