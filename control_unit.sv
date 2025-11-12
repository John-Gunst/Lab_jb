module control_unit (
    input  logic [31:0] instr,
    input logic stall_EX;
    output logic [3:0]  aluop,
    output logic        alusrc,
    output logic [1:0]  regsel,
    output logic        regwrite,
    output logic        gpio_we,
    //Decoded feilds to be passed to the CPU
    output logic [4:0]  rd,
    output logic [4:0]  rs1,
    output logic [4:0]  rs2,
    output logic [11:0] imm_i,
    output logic [19:0] imm_u,
    output logic [6:0]  opcode,
    output logic [4:0]  shamt,
    output logic stall_FETCH
);

    // === Decoded fields ===
    logic [2:0]  funct3;
    logic [6:0]  funct7;

    // === Instantiate the instruction decoder ===
    instructionDecoder my_decoder (
        .instr(instr),
        .opcode(opcode),
        .rd(rd),
        .funct3(funct3),
        .funct7(funct7),
        .rs1(rs1),
        .rs2(rs2),
        .imm_i(imm_i),
        .imm_u(imm_u),
        .shamt(shamt),
        .imm_j_extended(imm_j_extended),
        .imm_b_extended(imm_b_extended)
    );

    // === Main combinational control logic ===
    always_comb begin
        // Default values
        aluop    = 4'b0000;
        alusrc   = 1'b0;
        regsel   = 2'b00;
        regwrite = 1'b0;
        gpio_we  = 1'b0;
        stall_FETCH = 1'b0

        case (opcode)
            // ---------------- R-type (0110011)
            7'h33: begin
                regsel   = 2'b10;
            	regwrite = stall_EX? 1'b1: 1'b0;
                alusrc   = 1'b0;
                gpio_we  = 1'b0;

                case (funct3)
                    3'b000: aluop = (funct7 == 7'b0100000) ? 4'b0100 : 4'b0011; // add/sub
                    3'b111: aluop = 4'b0000; // and
                    3'b110: aluop = 4'b0001; // or
                    3'b100: aluop = 4'b0010; // xor
                    3'b001: aluop = 4'b1000; // sll
                    3'b101: aluop = (funct7 == 7'b0100000) ? 4'b1010 : 4'b1001; // sra/srl
                    3'b010: aluop = 4'b1100; // slt
                    3'b011: aluop = 4'b1101; // sltu
                    default: aluop = 4'b0000;
                endcase

                // RV32M (multiply) instructions
                if (funct7 == 7'b0000001) begin
                    case (funct3)
                        3'b000: aluop = 4'b0101; // mul
                        3'b001: aluop = 4'b0110; // mulh
                        3'b011: aluop = 4'b0111; // mulhu
                        default: ;

                    endcase
                end
            end

            // I-type (0010011)
            7'h13: begin
                regsel   = 2'b10;
            	regwrite = stall_EX? 1'b1: 1'b0;
                alusrc   = 1'b1;
                gpio_we  = 1'b0;
                case (funct3)
                    3'b000: aluop = 4'b0100; // BEQ, sub
                    3'b111: aluop = 4'b1101; // BGEU, sltu
                    3'b110: aluop = 4'b1101; // BLTU, sltu
                    3'b100: aluop = 4'b1100; // BLT, slt
                    3'b001: aluop = 4'b0100; // BNE, sub
                    3'b101: aluop = 4'b1100; // BGE, slt
                    default: aluop = 4'b0011;
                endcase
            end

            //  LUI (0110111)
            7'h37: begin
                regsel   = 2'b01;
            	regwrite = stall_EX? 1'b1: 1'b0;
                alusrc   = 1'b1;
                aluop    = 4'b0011;
                gpio_we  = 1'b0;
            end

            // SYSTEM (csrrw etc)
            7'h73: begin
                alusrc = 1'b0;
                aluop  = 4'b0000;
                if (imm_i == 12'hf02) begin
                    regsel   = 2'b00;
                    regwrite = 1'b0;
                    gpio_we  = 1'b1; // write GPIO
                end
                else if (imm_i == 12'hf00) begin
                    regsel   = 2'b00;
            	    regwrite = stall_EX? 1'b1: 1'b0;
                    gpio_we  = 1'b0; // read GPIO
                end
            end
            //J-Type
            7'h6F: begin
            	alusrc = 1'b1;
            	aluop = 4'b0011;
            	regwrite = stall_EX? 1'b1: 1'b0;
            	regsel = 2'b10;
            	gpio_we = 1'b0;
            end
            //B-Type
            7'h63: begin
            	//TODO: Determine alusrc code
            	case (funct3)
            	    3'b000: begin
                	aluop = 4'b0100; // ==
                    end
                    3'b001: begin
                	aluop = 4'b0100; // !=
                    end
                    3'b100: begin
                	aluop = 4'b1100; // < (signed)
                    end
                    3'b101: begin
                	aluop = 4'b1100; // >= (signed)
                    end
                    3'b110: begin
                	aluop = 4'b1101; // < (unsigned)
                    end
                    3'b111: begin
                	aluop = 4'b1101; // >= (unsigned)
                    end
		alusrc = 1'b0;
		regsel = 2'b00;
		regwrite = 1'b0;
		gpio_we = 1'b0;
	    end
            default: begin
                aluop    = 4'b0000;
                alusrc   = 1'b0;
                regsel   = 2'b00;
                regwrite = 1'b0;
                gpio_we  = 1'b0;
            end
        endcase
    end
endmodule
