// controller.sv
// I built this controller for the multicycle RISC-V processor.
// It has three sub-modules inside:
//   - mainsm   : the main FSM with 11 states
//   - aludec   : figures out what operation the ALU should do
//   - instrdec : figures out the immediate type from the opcode

module controller(
  input  logic        clk, reset,
  input  logic [6:0]  op,
  input  logic [2:0]  funct3,
  input  logic        funct7b5,
  input  logic        zero,
  output logic [1:0]  immsrc,
  output logic [1:0]  alusrca, alusrcb,
  output logic [1:0]  resultsrc,
  output logic        adrsrc,
  output logic [2:0]  alucontrol,
  output logic        irwrite, pcwrite,
  output logic        regwrite, memwrite
);
  logic [1:0] aluop;
  logic       branch, pcupdate;

  // PCWrite is 1 when we want to update the PC
  // either because of a normal update or a taken branch
  assign pcwrite = pcupdate | (branch & zero);

  mainsm fsm(
    .clk(clk), .reset(reset), .op(op),
    .alusrca(alusrca), .alusrcb(alusrcb),
    .resultsrc(resultsrc), .adrsrc(adrsrc),
    .aluop(aluop),
    .irwrite(irwrite), .pcupdate(pcupdate),
    .regwrite(regwrite), .memwrite(memwrite),
    .branch(branch)
  );

  aludec alu_dec(
    .opb5(op[5]), .funct3(funct3),
    .funct7b5(funct7b5), .aluop(aluop),
    .alucontrol(alucontrol)
  );

  instrdec instr_dec(
    .op(op), .immsrc(immsrc)
  );
endmodule

// Main FSM: Moore machine with 11 states
// I set all don't-care outputs to 0 so the testbench gives deterministic results
module mainsm(
  input  logic        clk, reset,
  input  logic [6:0]  op,
  output logic [1:0]  alusrca, alusrcb, resultsrc,
  output logic        adrsrc,
  output logic [1:0]  aluop,
  output logic        irwrite, pcupdate, regwrite, memwrite, branch
);
  typedef enum logic [3:0] {
    S0_FETCH    = 4'd0,
    S1_DECODE   = 4'd1,
    S2_MEMADR   = 4'd2,
    S3_MEMREAD  = 4'd3,
    S4_MEMWB    = 4'd4,
    S5_MEMWRITE = 4'd5,
    S6_EXECUTER = 4'd6,
    S7_ALUWB    = 4'd7,
    S8_EXECUTEI = 4'd8,
    S9_JAL      = 4'd9,
    S10_BEQ     = 4'd10
  } statetype;

  statetype state, nextstate;

  // state register resets to Fetch on reset
  always_ff @(posedge clk, posedge reset)
    if (reset) state <= S0_FETCH;
    else       state <= nextstate;

  // next state logic p.s. I look at the opcode to decide where to go after Decode
  always_comb
    case (state)
      S0_FETCH:   nextstate = S1_DECODE;
      S1_DECODE:
        case (op)
          7'b0000011: nextstate = S2_MEMADR;   // lw
          7'b0100011: nextstate = S2_MEMADR;   // sw
          7'b0110011: nextstate = S6_EXECUTER;  // R-type
          7'b0010011: nextstate = S8_EXECUTEI;  // I-type ALU
          7'b1101111: nextstate = S9_JAL;       // jal
          7'b1100011: nextstate = S10_BEQ;      // beq
          default:    nextstate = S0_FETCH;
        endcase
      S2_MEMADR:
        case (op)
          7'b0000011: nextstate = S3_MEMREAD;  // lw
          default:    nextstate = S5_MEMWRITE; // sw
        endcase
      S3_MEMREAD:  nextstate = S4_MEMWB;
      S4_MEMWB:    nextstate = S0_FETCH;
      S5_MEMWRITE: nextstate = S0_FETCH;
      S6_EXECUTER: nextstate = S7_ALUWB;
      S7_ALUWB:    nextstate = S0_FETCH;
      S8_EXECUTEI: nextstate = S7_ALUWB;
      S9_JAL:      nextstate = S7_ALUWB;
      S10_BEQ:     nextstate = S0_FETCH;
      default:     nextstate = S0_FETCH;
    endcase

  // output logic p.s. I set all signals to 0 first, then override for each state
  always_comb begin
    // I set everything to 0 by default so don't-care signals are always 0
    alusrca  = 2'b00;
    alusrcb  = 2'b00;
    resultsrc= 2'b00;
    adrsrc   = 1'b0;
    aluop    = 2'b00;
    irwrite  = 1'b0;
    pcupdate = 1'b0;
    regwrite = 1'b0;
    memwrite = 1'b0;
    branch   = 1'b0;

    case (state)
      // Fetch: read the instruction from memory and compute PC+4
      S0_FETCH: begin
        adrsrc   = 1'b0;
        irwrite  = 1'b1;
        alusrca  = 2'b00;   // PC
        alusrcb  = 2'b10;   // 4
        aluop    = 2'b00;   // add
        resultsrc= 2'b10;   // ALUResult → PCNext
        pcupdate = 1'b1;
      end

      // Decode: compute PCTarget in case we need it for a branch or jump
      S1_DECODE: begin
        alusrca  = 2'b01;   // OldPC
        alusrcb  = 2'b01;   // ImmExt
        aluop    = 2'b00;   // add
      end

      // MemAdr: calculate the memory address (rs1 + immediate)
      S2_MEMADR: begin
        alusrca  = 2'b10;   // A (rs1)
        alusrcb  = 2'b01;   // ImmExt
        aluop    = 2'b00;   // add
      end

      // MemRead: read data from memory using ALUOut as address
      S3_MEMREAD: begin
        resultsrc= 2'b00;
        adrsrc   = 1'b1;    // ALUOut as memory address
      end

      // MemWB: write the data we loaded from memory into the register file
      S4_MEMWB: begin
        resultsrc= 2'b01;   // ReadData
        regwrite = 1'b1;
      end

      // MemWrite: write data to memory
      S5_MEMWRITE: begin
        resultsrc= 2'b00;
        adrsrc   = 1'b1;
        memwrite = 1'b1;
      end

      // ExecuteR: do the ALU operation using two registers (rs1 and rs2)
      S6_EXECUTER: begin
        alusrca  = 2'b10;   // A (rs1)
        alusrcb  = 2'b00;   // B (rs2)
        aluop    = 2'b10;   // R-type op (funct3/funct7 decode)
      end

      // ALUWB: write the ALU result back to the register file
      S7_ALUWB: begin
        resultsrc= 2'b00;   // ALUOut
        regwrite = 1'b1;
      end

      // ExecuteI: do the ALU operation using a register and an immediate
      S8_EXECUTEI: begin
        alusrca  = 2'b10;   // A (rs1)
        alusrcb  = 2'b01;   // ImmExt
        aluop    = 2'b10;   // I-type op (funct3 decode)
      end

      // JAL: update PC to jump target, and save PC+4 in ALUOut for the return address
      S9_JAL: begin
        alusrca  = 2'b01;   // OldPC
        alusrcb  = 2'b10;   // 4
        aluop    = 2'b00;   // add
        resultsrc= 2'b00;   // ALUOut (= OldPC+imm from decode)
        pcupdate = 1'b1;    // PC ← ALUOut
      end

      // BEQ: subtract rs1-rs2, if result is zero then take the branch
      S10_BEQ: begin
        alusrca  = 2'b10;   // A (rs1)
        alusrcb  = 2'b00;   // B (rs2)
        aluop    = 2'b01;   // subtract
        resultsrc= 2'b00;
        branch   = 1'b1;    // PCWrite = branch & zero
      end

      default: ; // all zero
    endcase
  end
endmodule

// ALU Decoder
// I look at ALUOp and funct3 to decide what operation the ALU should do.
// The encoding I used matches the multicycle ALU from the textbook:
//   010 = add,  110 = subtract,  000 = AND,  001 = OR,  111 = SLT
module aludec(
  input  logic        opb5,
  input  logic [2:0]  funct3,
  input  logic        funct7b5,
  input  logic [1:0]  aluop,
  output logic [2:0]  alucontrol
);
  logic RtypeSub;
  assign RtypeSub = funct7b5 & opb5; // this is only true for the R-type sub instruction

  always_comb
    case (aluop)
      2'b00: alucontrol = 3'b010; // always add for lw, sw, fetch, decode states
      2'b01: alucontrol = 3'b110; // always subtract for beq
      default: // for R-type and I-type, I check funct3 to pick the right operation
        case (funct3)
          3'b000: alucontrol = RtypeSub ? 3'b110 : 3'b010; // sub / add,addi
          3'b010: alucontrol = 3'b111; // slt, slti
          3'b110: alucontrol = 3'b001; // or, ori
          3'b111: alucontrol = 3'b000; // and, andi
          default: alucontrol = 3'b000; // set to 0 for don't care
        endcase
    endcase
endmodule

// Instruction Decoder
// I use the opcode to figure out what type of immediate the instruction uses
// and set ImmSrc accordingly so the extend unit knows how to sign-extend it
module instrdec(
  input  logic [6:0] op,
  output logic [1:0] immsrc
);
  always_comb
    case (op)
      7'b0110011: immsrc = 2'b00; // R-type (no immediate, set to 0)
      7'b0010011: immsrc = 2'b00; // I-type ALU
      7'b0000011: immsrc = 2'b00; // lw
      7'b0100011: immsrc = 2'b01; // sw
      7'b1100011: immsrc = 2'b10; // beq
      7'b1101111: immsrc = 2'b11; // jal
      default:    immsrc = 2'b00; // default to 0
    endcase
endmodule
