module instructionDecoder(
    input  logic [31:0] instr,
    output logic [6:0]  opcode,
    output logic [4:0]  rd,
    output logic [2:0]  funct3,
    output logic [6:0]  funct7,
    output logic [4:0]  rs1,
    output logic [4:0]  rs2,
    output logic [11:0] imm_i,
    output logic [19:0] imm_u,
    output logic [4:0]  shamt,
    output logic [31:0] imm_j_extended,
    output logic [31:0] imm_b_extended
);

    assign opcode = instr[6:0];
    assign rd     = instr[11:7];
    assign funct3 = instr[14:12];
    assign rs1    = instr[19:15];
    assign rs2    = instr[24:20];
    assign funct7 = instr[31:25];
    assign imm_i  = instr[31:20];
    assign imm_u  = instr[31:12];
    assign shamt  = instr[24:20]; // for slli/srli/srai
    
endmodule
