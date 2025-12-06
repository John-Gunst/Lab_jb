module instructionDecoder (
    input  logic [31:0] instruction,

    output logic [6:0]  op,
    output logic [2:0]  funct3,
    output logic [6:0]  funct7,
    output logic [4:0]  rd,
    output logic [4:0]  rs1,
    output logic [4:0]  rs2,

    // I-type / CSR / U-type
    output logic [11:0] imm,
    output logic [31:0] imm_i_sext,

    output logic [19:0] imm_u,
    output logic [11:0] csr_address,

    // B-type and J-type immediates (sign-extended)
    output logic [31:0] imm_b_sext,  // branch offset
    output logic [31:0] imm_j_sext   // jump offset (JAL)
);

    // Local helper: build sign-extended B-type immediate from instruction
    function automatic logic [31:0] make_branch_imm(input logic [31:0] inst);
        make_branch_imm = {{19{inst[31]}},  // sign extension
                           inst[31],        // imm[12]
                           inst[7],         // imm[11]
                           inst[30:25],     // imm[10:5]
                           inst[11:8],      // imm[4:1]
                           1'b0};           // imm[0]
    endfunction

    // Local helper: build sign-extended J-type immediate from instruction
    function automatic logic [31:0] make_jump_imm(input logic [31:0] inst);
        make_jump_imm = {{11{inst[31]}},  // sign extension
                         inst[31],        // imm[20]
                         inst[19:12],     // imm[19:12]
                         inst[20],        // imm[11]
                         inst[30:21],     // imm[10:1]
                         1'b0};           // imm[0]
    endfunction

    always_comb begin
        op        = instruction[6:0];
        rd        = instruction[11:7];
        funct3    = instruction[14:12];
        rs1       = instruction[19:15];
        rs2       = instruction[24:20];
        funct7    = instruction[31:25];

        // CSR & I-type (standard 12-bit immediate)
        csr_address = instruction[31:20];
        imm         = instruction[31:20];
        imm_i_sext  = {{20{instruction[31]}}, instruction[31:20]};

        // U-type (LUI/AUIPC)
        imm_u       = instruction[31:12];

        // B-type & J-type immediates via helper functions
        imm_b_sext = make_branch_imm(instruction);
        imm_j_sext = make_jump_imm(instruction);
    end

endmodule
