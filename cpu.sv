// cpu.sv - updated (regfile instantiation moved after EX->WB declarations)
module cpu (

    input  logic         clk,
    input  logic         rst,       // active-high reset
    input  logic [31:0]  gpio_in,   // io0 (switches)
    output logic [31:0]  gpio_out   // io2 (hex display)
);

 logic [31:0] instmem [0:4095];

// Initialize instruction memory
initial begin
    integer i;
    // Set all memory locations to 0 to avoid 'X' fetches
    for (i = 0; i < 4096; i = i + 1)
        instmem[i] = 32'd0;

    // Load instruction memory from file
    $readmemh("instmem.dat", instmem);
    `ifndef SYNTHESIS
    `ifndef ALTERA_RESERVED_QIS
	    $display("[%0t] Instruction memory initialized from instmem.dat", $time);
    `endif
    `endif
 end
// ==========================================================
// Program Counter + Fetch Register
// ==========================================================
logic [31:0] instr_F;      // instruction fetched (registered)
logic [11:0] pc, pc_next;  // 12 bits for 4K word addresses
logic [11:0] branch_addr, jal_addr, jalr_addr;

// ==========================================================
// Synchronous PC Update + Fetch
// ==========================================================
logic halt_pending;

always_ff @(posedge clk) begin
    if (rst) begin
        pc <= 12'd0;
        instr_F <= 32'd0;
        halt_pending <= 1'b0;
        `ifndef SYNTHESIS
        `ifndef ALTERA_RESERVED_QIS
            $display("[%0t] CPU RESET asserted: pc <= 0", $time);
        `endif
        `endif
    end
    else if (!stall_FETCH) begin
        // Fetch instruction from memory
        instr_F <= instmem[pc];
        
        // Debug printout
        `ifndef SYNTHESIS
        `ifndef ALTERA_RESERVED_QIS
            $display("[%0t] FETCH: pc=0x%03h -> instr=0x%08h", $time, pc, instmem[pc]);
        `endif
        `endif
        // Update PC
        //TODO add pcsrc case
        pc_next = pc + 12'd1;
        pc <= pc_next;
    end
    else begin
    	//stall
    end
end

    // ------------------------------------------------------------------
    // Control unit (combinational)
    // ------------------------------------------------------------------
    // Control signals
    logic [3:0]  aluop;
    logic        alusrc;
    logic [1:0]  regsel;
    logic        regwrite;
    logic        gpio_we;
    //Fields to be passed to registers
    logic [4:0]  rd;
    logic [4:0]  rs1;
    logic [4:0]  rs2;
    logic [11:0] imm_i;
    logic [19:0] imm_u; 
    logic [6:0] opcode;


    control_unit my_control_unit(
	    .instr   (instr_F),
	    .aluop   (aluop),
	    .alusrc  (alusrc),
	    .regsel  (regsel),
	    .regwrite(regwrite),
	    .gpio_we (gpio_we),
	    .rd      (rd),
	    .rs1     (rs1),
	    .rs2     (rs2),
	    .imm_i   (imm_i),
	    .imm_u   (imm_u),
	    .opcode  (opcode),
	    .stall_EX(stall_EX),
	    .stall_FETCH(stall_FETCH)
    );

    // ------------------------------------------------------------------
    // Register file signals (declare early)
    // ------------------------------------------------------------------
    logic [31:0] rf_readdata1, rf_readdata2;
    logic rf_we;
    logic [4:0] rf_waddr;
    logic [31:0] rf_wdata;

    // ------------------------------------------------------------------
    // ALU signals (declared early, ALU instantiated later)
    // ------------------------------------------------------------------
    logic [31:0] alu_inA, alu_inB, alu_R;
    logic alu_zero;

    // ------------------------------------------------------------------
    // CSR mapping (combinational read)
    // ------------------------------------------------------------------
    logic [31:0] csr_out;
    always_comb begin
        csr_out = 32'd0;
        if (imm_i == 12'hf00) begin
            csr_out = gpio_in;
        end else if (imm_i == 12'hf02) begin
            csr_out = 32'd0;
        end else begin
            csr_out = 32'd0;
        end
    end

    // ------------------------------------------------------------------
    // EX -> WB pipeline registers (declare BEFORE we instantiate regfile if we want to wire them)
    // ------------------------------------------------------------------
    logic [31:0] ex_alu_R_q;
    logic [4:0]  ex_rd_q;
    logic        ex_regwrite_q;
    logic [1:0]  ex_regsel_q;
    logic        ex_gpio_we_q;
    logic [11:0] ex_csr_addr_q;
    logic [19:0] ex_imm_u_q;
    logic [31:0] ex_csr_read_q;
    logic [31:0] ex_rs1_val_q;
    logic [1:0] pcsrc_EX;
    logic [11:0] PC_EX;
     //hazard detection registers
    logic stall_EX;
    logic isload_EX;
    logic stall_FETCH;
 
  
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            ex_rd_q        <= 5'd0;
            ex_gpio_we_q    <= 1'b0;
            ex_csr_read_q  <= 32'd0;
            ex_imm_u_q     <= 20'd0;
            pcsrc_EX         <= 1'b0;
            PC_EX          <= 12'b0;
      
        end else begin
             `ifndef SYNTHESIS
	    // print the fetch/decode/register state for last two cycles
	    $display("[%0t] DBG: pc=%0h instr_F=0x%08h ctrl_rd=%0d rd_field=%0d ex_rd_q=%0d alu_R=0x%08h",
		     $time, pc, instr_F, rd, instr_F[11:7], ex_rd_q, alu_R);
	    `endif
            stall_EX <= stall_FETCH
            ex_rd_q        <= rd;
            ex_csr_read_q  <= rf_readdata1;
            ex_gpio_we_q    <= gpio_we;
            ex_imm_u_q     <= imm_u;
            PC_EX          <= pc;
        end
    end
        assign ex_rs1_val_q = rf_readdata1;
	assign ex_alu_R_q = alu_R;
	assign ex_regsel_q = regsel;
	assign ex_regwrite_q = regwrite;
	assign ex_csr_addr_q  = imm_u;


    // ------------------------------------------------------------------
    // REGFILE INSTANTIATION
    // Moved here so ex_*_q signals are declared before use
    // You can temporarily wire writeaddr/writedata to ex_*_q for debugging experiments.
    // ------------------------------------------------------------------

    // === Normal, safe WB-registered wiring ===
   regfile u_regfile (
        .clk (clk),
        .we  (rf_we),
        .readaddr1 (rs1),
        .readaddr2 (rs2),
        .writeaddr (ex_rd_q),
        .writedata (rf_wdata),
        .readdata1 (rf_readdata1),
        .readdata2 (rf_readdata2)
    );
    
    //resolve pcsrc_EX TODO Finish
    //case(opcode):


    // ------------------------------------------------------------------
    // ALU instantiation (external module)
    // expect alu(A,B,op,R,zero)
    // ------------------------------------------------------------------
    alu u_alu (
        .A (alu_inA),
        .B (alu_inB),
        .op (aluop),
        .R (alu_R),
        .zero (alu_zero)
    );

   // ALU input selection
   logic [31:0] signext_imm_i;

   assign signext_imm_i = { {20{imm_i[11]}}, imm_i };

   // For LUI (opcode = 0x37), rs1 is ignored → force alu_inA = 0
   always_comb begin
       if (opcode == 7'h37) begin
           alu_inA = 32'b0;  // LUI
       end else begin
           alu_inA = rf_readdata1;
       end

       alu_inB = (alusrc) ? signext_imm_i : rf_readdata2;
   end
  // WRITEBACK

	always_ff @(posedge clk) begin
	    if (gpio_we) begin 
	    	gpio_out <= rf_readdata1;
	    end
	end
	always_ff @(posedge clk or posedge rst) begin
	    if (rst) begin
		rf_we      <= 1'b0;
		rf_waddr   <= 5'd0;
		rf_wdata   <= 32'd0;
		//gpio_out   <= 32'd0;

	    end else begin
		// Default values each cycle
		rf_we      <= 1'b0;
		rf_waddr   <= 5'd0;
		rf_wdata   <= 32'd0;

		// -------- Register File Writeback --------
		if (ex_regwrite_q) begin
		    rf_we    <= 1'b1;
		    rf_waddr <= rd;//ex_rd_q;

		     case (ex_regsel_q)
			2'b00: rf_wdata <= csr_out;
		        2'b01: rf_wdata <= {imm_u, 12'b0};   // U-type (LUI/AUIPC)
		        2'b10: rf_wdata <= ex_alu_R_q;            // ALU result
		        2'b11: rf_wdata <= {20'b0, PC_EX};
		        default: rf_wdata <= 32'd0;
		    endcase

		    `ifndef SYNTHESIS
		    `ifndef ALTERA_RESERVED_QIS
		        $display("[%0t] WB: write x%0d <= 0x%08h (signed %0d), csr_out=0x%08h,  ex_regsel_q=x%0d, ex_regwrite_q=%0d",
		                 $time, ex_rd_q, rf_wdata, $signed(rf_wdata), csr_out, ex_regsel_q, rf_we);
		    `endif
		    `endif
		end
	    end
	end
endmodule
