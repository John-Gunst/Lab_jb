// regsel: 00=GPIO/CSR read, 01=U-imm (LUI), 10=ALU, 11=PC+4 (JAL/JALR link)
// framos and jgunst
module control_unit (
    input  logic [6:0]  funct7,
    input  logic [2:0]  funct3,
    input  logic [6:0]  opcode,
    input  logic [11:0] imm_i,     // imm[11:0] (CSR addr and I-shifts)

    // stall from EX stage (e.g., hazard / memory wait)
    input  logic        stall_EX,

    output logic        alusrc,
    output logic        regwrite,
    output logic [1:0]  regsel,    // 00: GPIO/CSR, 01: U-imm, 10: ALU, 11: PC+4
    output logic [3:0]  aluop,
    output logic        gpio_we,

    // control for B / J type
    output logic        branch,    // B-type instruction
    output logic        jal,       // JAL
    output logic        jalr,      // JALR

    // stall signal back to fetch stage
    output logic        stall_FETCH
);

    // Match ALU encodings from alu.sv
    localparam [3:0]
        ALU_AND   = 4'b0000,
        ALU_OR    = 4'b0001,
        ALU_XOR   = 4'b0010,
        ALU_ADD   = 4'b0011,
        ALU_SUB   = 4'b0100,
        ALU_MUL   = 4'b0101,  
        ALU_MULH  = 4'b0110,  // high 32 (signed*signed)
        ALU_MULHU = 4'b0111,  // high 32 (unsigned*unsigned)
        ALU_SLL   = 4'b1000,
        ALU_SRL   = 4'b1001,
        ALU_SRA   = 4'b1010,  
        ALU_SRA2  = 4'b1011, 
        ALU_SLT   = 4'b1100,
        ALU_SLTU  = 4'b1101;

    localparam [1:0]
        REGSEL_GPIO = 2'b00,
        REGSEL_U    = 2'b01,
        REGSEL_ALU  = 2'b10,
        REGSEL_PC4  = 2'b11;   // write PC+4 to rd (JAL/JALR)

    logic is_srai, is_srli, is_slli;
    assign is_slli = (funct3 == 3'b001) && (imm_i[11:5] == 7'b0000000);
    assign is_srli = (funct3 == 3'b101) && (imm_i[11:5] == 7'b0000000);
    assign is_srai = (funct3 == 3'b101) && (imm_i[11:5] == 7'b0100000);

    always_comb begin
        // safe defaults
        alusrc      = 1'b0;
        regwrite    = 1'b0;
        regsel      = REGSEL_ALU;
        aluop       = ALU_ADD;
        gpio_we     = 1'b0;

        // new defaults for control-flow
        branch      = 1'b0;
        jal         = 1'b0;
        jalr        = 1'b0;

        // default: fetch should stall whenever EX says so
        stall_FETCH = stall_EX;

        unique case (opcode)

            // ---------------- R-type ALU (0110011) ----------------
            7'b0110011: begin
                alusrc   = 1'b0;         // rs2
                regwrite = 1'b1;
                regsel   = REGSEL_ALU;

                unique case (funct3)
                    3'b000: aluop = (funct7[5]) ? ALU_SUB : ALU_ADD;  // SUB/ADD
                    3'b001: aluop = ALU_SLL;                          // SLL
                    3'b010: aluop = ALU_SLT;                          // SLT
                    3'b011: aluop = ALU_SLTU;                         // SLTU
                    3'b100: aluop = ALU_XOR;                          // XOR
                    3'b101: aluop = (funct7[5]) ? ALU_SRA : ALU_SRL;  // SRA/SRL
                    3'b110: aluop = ALU_OR;                           // OR
                    3'b111: aluop = ALU_AND;                          // AND
                    default: ;
                endcase

                // M-extension (funct7=0000001)
                if (funct7 == 7'b0000001) begin
                    unique case (funct3)
                        3'b000: aluop = ALU_MUL;    // MUL
                        3'b001: aluop = ALU_MULH;   // MULH (signed*signed)
                        3'b011: aluop = ALU_MULHU;  // MULHU (unsigned*unsigned)
                        default: ;
                    endcase
                end
            end

            // ---------------- I-type ALU (0010011) ----------------
            7'b0010011: begin
                alusrc   = 1'b1;         // imm (I-type)
                regwrite = 1'b1;
                regsel   = REGSEL_ALU;

                unique case (funct3)
                    3'b000: aluop = ALU_ADD;       // ADDI
                    3'b010: aluop = ALU_SLT;       // SLTI
                    3'b011: aluop = ALU_SLTU;      // SLTIU
                    3'b100: aluop = ALU_XOR;       // XORI
                    3'b110: aluop = ALU_OR;        // ORI
                    3'b111: aluop = ALU_AND;       // ANDI
                    3'b001: if (is_slli) aluop = ALU_SLL;             // SLLI
                    3'b101: begin
                        if (is_srli)      aluop = ALU_SRL;            // SRLI
                        else if (is_srai) aluop = ALU_SRA;            // SRAI
                    end
                    default: ;
                endcase
            end

            // ---------------- LUI (0110111) ----------------
            7'b0110111: begin
                regwrite = 1'b1;
                regsel   = REGSEL_U;     // write U-imm to rd
                // alusrc/aluop don't-care
            end

            // ---------------- B-type branches (1100011) ----------------
            // Uses rs1, rs2 and immediate from decoder (imm_b_sext).
            // Branch unit will look at 'branch', funct3 and ALU result.
            7'b1100011: begin
                branch   = 1'b1;
                alusrc   = 1'b0;         // compare rs1 vs rs2
                regwrite = 1'b0;         // branches don't write rd
                regsel   = REGSEL_ALU;   // don't care for writeback

                unique case (funct3)
                    3'b000, 3'b001: aluop = ALU_SUB;   // BEQ/BNE use SUB → zero flag
                    3'b100, 3'b101: aluop = ALU_SLT;   // BLT/BGE use signed < via SLT
                    3'b110, 3'b111: aluop = ALU_SLTU;  // BLTU/BGEU use unsigned < via SLTU
                    default: ;
                endcase
            end

            // ---------------- JAL (1101111) ----------------
            7'b1101111: begin
                jal      = 1'b1;
                regwrite = 1'b1;         // write link register
                regsel   = REGSEL_PC4;   // PC+4 to rd
                // alusrc/aluop are don't-care for datapath ALU
            end

            // ---------------- JALR (1100111) ----------------
            7'b1100111: begin
                if (funct3 == 3'b000) begin
                    jalr     = 1'b1;
                    regwrite = 1'b1;     // write link register
                    regsel   = REGSEL_PC4; 
                    // PC target = rs1 + imm_i_sext in PC logic
                end
            end

            // ---------------- CSR / GPIO (1110011) ----------------
            7'b1110011: begin
                // only CSRRW (funct3=001)
                if (funct3 == 3'b001) begin
                    // csrrw HEX (io2): imm12 = 12'hf02 → write HEX, no RF write
                    if (imm_i == 12'hf02) begin
                        gpio_we  = 1'b1;
                        regwrite = 1'b0;
                    end
                    // csrrw io1 (buttons): imm12 = 12'hf00 → read, write rd
                    else if (imm_i == 12'hf00) begin
                        regwrite = 1'b1;
                        regsel   = REGSEL_GPIO;
                    end
                end
            end

            default: ; // keep safe defaults
        endcase

        // ------------- Stall handling: no side-effects ---------------
        if (stall_EX) begin
            // Treat as a NOP for this cycle from the perspective of new effects.
            regwrite = 1'b0;
            gpio_we  = 1'b0;
            // Optional: suppress new control-flow changes while stalled
            branch   = 1'b0;
            jal      = 1'b0;
            jalr     = 1'b0;
            // stall_FETCH already = stall_EX
        end
    end
endmodule
