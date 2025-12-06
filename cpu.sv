// framos and jgunst
module cpu(
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] io0_in,        // switches (read when regsel=2'b00)
    output logic        gpio_we,       // asserted by CU for HEX writes (csrrw io2)
    output logic [31:0] gpio_wdata     // data to HEX (from rs1)
);

    // ------------------ INSTRUCTION MEMORY (F) ------------------
    logic [31:0] inst_ram [0:4095];
    initial $readmemh("test.dat", inst_ram);

    // Fetch-stage PC (byte address)
    logic [31:0] PC_FETCH;

    // Instruction fetched in F (combinational read)
    logic [31:0] instr_F;
    assign instr_F = inst_ram[PC_FETCH[13:2]];  // word-aligned

    // EX-stage PC (PC of instruction_EX)
    logic [31:0] PC_EX;

    // Next PC + helpers
    logic [31:0] pc_next;
    logic [31:0] pc_plus4_F;
    logic [31:0] pc_plus4_EX;

    assign pc_plus4_F  = PC_FETCH + 32'd4;
    assign pc_plus4_EX = PC_EX    + 32'd4;

    // ------------------ F->EX PIPELINE REGS ------------------
    logic [31:0] instruction_EX;

    // ------------------ DECODER OUTPUTS ------------------
    logic [6:0]  op;
    logic [2:0]  funct3;
    logic [6:0]  funct7;
    logic [4:0]  rd, rs1, rs2;
    logic [11:0] imm;         
    logic [31:0] imm_i_sext;   
    logic [19:0] imm_u;        
    logic [11:0] csr_address;

    logic [31:0] imm_b_sext;   // for B-type
    logic [31:0] imm_j_sext;   // for J-type

    instructionDecoder instructionDecoder_i (
        .instruction(instruction_EX),
        .op(op), 
        .funct3(funct3), 
        .funct7(funct7),
        .rd(rd), 
        .rs1(rs1), 
        .rs2(rs2),
        .imm(imm), 
        .imm_i_sext(imm_i_sext), 
        .imm_u(imm_u),
        .csr_address(csr_address),
        .imm_b_sext(imm_b_sext),
        .imm_j_sext(imm_j_sext)
    );

    // ------------------ STALL SIGNALS ------------------
    // In a full design, stall_EX would come from a hazard unit / memory.
    // For now we tie it off; wiring is in place.
    logic stall_EX;
    assign stall_EX = 1'b0;

    logic stall_FETCH;   // output of CU, used to stall PC

    // ------------------ CONTROL (EX) ------------------
    logic       alusrc;
    logic       regwrite;
    logic [1:0] regsel_EX;   // 00: GPIO, 01: U-imm, 10: ALU, 11: PC+4
    logic [3:0] aluop;

    // branch/jump bits from CU
    logic branch;
    logic jal;
    logic jalr;

    control_unit cu_i (
        .funct7     (funct7),
        .funct3     (funct3),
        .opcode     (op),
        .imm_i      (imm),
        .stall_EX   (stall_EX),    // new input

        .alusrc     (alusrc),
        .regwrite   (regwrite),
        .regsel     (regsel_EX),
        .aluop      (aluop),
        .gpio_we    (gpio_we),

        .branch     (branch),
        .jal        (jal),
        .jalr       (jalr),

        .stall_FETCH(stall_FETCH)  // new output
    );

    // ------------------ ALU (EX) ------------------
    logic [31:0] rs1_data_EX, rs2_data_EX;
    logic [31:0] alu_input2, alu_result_EX;
    logic        alu_zero_EX;

    assign alu_input2 = alusrc ? imm_i_sext : rs2_data_EX;

    alu alu_inst (
        .A   (rs1_data_EX),
        .B   (alu_input2),
        .op  (aluop),
        .R   (alu_result_EX),
        .zero(alu_zero_EX)
    );

    // ------------------ BRANCH / JUMP ADDRESSES (EX) ------------------
    logic [31:0] branch_addr_EX;
    logic [31:0] jal_addr_EX;
    logic [31:0] jalr_addr_EX;

    assign branch_addr_EX = PC_EX + imm_b_sext;
    assign jal_addr_EX    = PC_EX + imm_j_sext;
    assign jalr_addr_EX   = (rs1_data_EX + imm_i_sext) & ~32'd1;

    // ------------------ BRANCH RESOLUTION (uses ALU R_EX) -------------
    logic branch_taken_EX;

    always_comb begin
        branch_taken_EX = 1'b0;
        if (branch) begin
            unique case (funct3)
                // BEQ: SUB → branch if R_EX == 0
                3'b000: branch_taken_EX = (alu_result_EX == 32'd0);
                // BNE: SUB → branch if R_EX != 0
                3'b001: branch_taken_EX = (alu_result_EX != 32'd0);
                // BLT: SLT → branch if R_EX == 1
                3'b100: branch_taken_EX = (alu_result_EX == 32'd1);
                // BGE: SLT → branch if R_EX == 0
                3'b101: branch_taken_EX = (alu_result_EX == 32'd0);
                // BLTU: SLTU → branch if R_EX == 1
                3'b110: branch_taken_EX = (alu_result_EX == 32'd1);
                // BGEU: SLTU → branch if R_EX == 0
                3'b111: branch_taken_EX = (alu_result_EX == 32'd0);
                default: ;
            endcase
        end
    end

    // ------------------ PC SRC (pcsrc_EX) & NEXT PC -------------------
    // 00: PC+4 (normal)
    // 01: branch_addr_EX
    // 10: jal_addr_EX
    // 11: jalr_addr_EX
    logic [1:0] pcsrc_EX;

    always_comb begin
        pcsrc_EX = 2'b00;           // default: sequential
        if (jal) begin
            pcsrc_EX = 2'b10;
        end else if (jalr) begin
            pcsrc_EX = 2'b11;
        end else if (branch && branch_taken_EX) begin
            pcsrc_EX = 2'b01;
        end
    end

    always_comb begin
        unique case (pcsrc_EX)
            2'b00: pc_next = pc_plus4_F;
            2'b01: pc_next = branch_addr_EX;
            2'b10: pc_next = jal_addr_EX;
            2'b11: pc_next = jalr_addr_EX;
            default: pc_next = pc_plus4_F;
        endcase
    end

    // Flush the instruction in EX when control flow changes
    logic flush_EX;
    assign flush_EX = (pcsrc_EX != 2'b00);

    // ------------------ EX->WB PIPELINE ------------------
    logic [1:0]  regsel_WB;
    logic [31:0] alu_result_WB;
    logic        regwrite_WB;
    logic [4:0]  rd_WB;
    logic [19:0] imm_u_WB;
    logic [31:0] pc4_WB;         // PC_EX + 4 for JAL/JALR link

    // ------------------ WRITEBACK MUX (WB) ------------------
    logic [31:0] rd_data_WB;
    always_comb begin
        unique case (regsel_WB)
            2'd0: rd_data_WB = io0_in;              // CSRRW SW readback
            2'd1: rd_data_WB = {imm_u_WB,12'b0};    // LUI (upper 20 << 12)
            2'd2: rd_data_WB = alu_result_WB;       // ALU result
            2'd3: rd_data_WB = pc4_WB;              // PC+4 (JAL/JALR link)
            default: rd_data_WB = 32'b0;
        endcase
    end

    // ------------------ REGISTER FILE ------------------
    regfile reg_file (
        .clk       (clk),
        .we        (regwrite_WB),   // commit-stage write enable
        .readaddr1 (rs1),
        .readaddr2 (rs2),
        .writeaddr (rd_WB),
        .writedata (rd_data_WB),
        .readdata1 (rs1_data_EX),
        .readdata2 (rs2_data_EX)
    );

    assign gpio_wdata = rs1_data_EX;

    // ------------------ FETCH (F) ------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            PC_FETCH <= 32'd0;
        end else if (!stall_FETCH) begin
            PC_FETCH <= pc_next;
        end
        // if stall_FETCH==1, hold PC_FETCH
    end

    // ------------------ F->EX PIPELINE TRANSFER ------------------
    localparam [31:0] NOP_INST = 32'h00000013; // ADDI x0, x0, 0

    always_ff @(posedge clk) begin
        if (!rst_n) begin
            instruction_EX <= NOP_INST;
            PC_EX          <= 32'd0;
        end else if (!stall_EX) begin
            if (flush_EX) begin
                // Inject bubble after taken branch / jump
                instruction_EX <= NOP_INST;
                PC_EX          <= 32'd0;
            end else begin
                instruction_EX <= instr_F;
                PC_EX          <= PC_FETCH;
            end
        end
        // if stall_EX==1, hold current EX-stage instruction and PC_EX
    end

    // ------------------ EX->WB PIPELINE TRANSFER ------------------
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            regsel_WB     <= 2'd0;
            alu_result_WB <= 32'd0;
            regwrite_WB   <= 1'b0;
            rd_WB         <= 5'd0;
            imm_u_WB      <= 20'd0;
            pc4_WB        <= 32'd0;
        end else if (!stall_EX) begin
            regsel_WB     <= regsel_EX;
            alu_result_WB <= alu_result_EX;
            regwrite_WB   <= regwrite;
            rd_WB         <= rd;
            imm_u_WB      <= imm_u;
            pc4_WB        <= pc_plus4_EX; // value written by JAL/JALR
        end
        // if stall_EX==1, hold current WB-stage control/data
    end

endmodule
