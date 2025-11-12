/* Copyright 2020 Jason Bakos, Philip Conrad, Charles Daniels */

/* Top-level module for CSCE611 RISC-V CPU, for running under simulation.  In
 * this case, the I/Os and clock are driven by the simulator. */

module simtop;
   logic [31:0] instr;
   logic [3:0]  aluop;
   logic        alusrc;
   logic [1:0]  regsel;
   logic        regwrite;
   logic        gpio_we;
   logic [4:0]  rd;
   logic [4:0]  rs1;
   logic [4:0]  rs2;
   logic [11:0] imm_i;
   logic [19:0] imm_u;
   
   control_unit dut (
        .instr(instr),
        .aluop(aluop),
        .alusrc(alusrc),
        .regsel(regsel),
        .regwrite(regwrite),
        .gpio_we(gpio_we),
        .rd(rd),
    	.rs1(rs1),
    	.rs2(rs2),
    	.imm_i(imm_i),
    	.imm_u(imm_u),
    	.opcode(opcode),
        .shamt(shamt),
        .imm_j_extended(imm_j_extended),
        .imm_b_extended(imm_b_extended)
    );

    // Test vector memory
    logic [31:0] instmem [0:255];    // instructions
    logic [8:0]  expected [0:255];   // expected outputs (9 bits)
    integer num_tests = 7;
    integer i;
    logic [8:0] actual;
    initial begin
        // Load test vectors
        $readmemh("instmem.dat", instmem);
     	//vector aluop, alusrc, regsel, regwrite, gpio_we = 0
        $readmemb("expected.dat", expected);
	
	$display("Starting Control Unit Test Bench");
   	for (i = 0; i < num_tests; i++) begin
            instr = instmem[i];
            #1; // Wait for combinational logic to settle

            actual = {aluop, alusrc, regsel, regwrite, gpio_we};

            if (actual !== expected[i]) begin
                $display("X Mismatch at test %0d", i);
                $display("   Instr:    %b", instr);
                $display("   Expected: %b", expected[i]);
                $display("   Got:      %b", actual);
         	$display("   aluop =   %b", aluop);
         	$display("   alusrc=   %b", alusrc);
         	$display("   regsel =  %b", regsel);
         	$display("   regwrite= %b", regwrite);
         	$display("   gpio_we = %b", gpio_we);
            end else begin
                $display("✅ Test %0d passed: %b", i, instr);
            end
        end

        $display("Control Unit Testing Complete");
    end
endmodule

