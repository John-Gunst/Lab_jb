module simtop;

   logic [31:0] instr;
   logic stall_EX;
   logic [31:0] R_EX;
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
   logic [1:0] pcsrc_EX;
   logic stall_FETCH;
   
   control_unit dut (
        .instr(instr),
        .stall_EX(stall_EX),
        .R_EX(R_EX),
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
    	.pcsrc_EX(pcsrc_EX),
    	.stall_FETCH(stall_FETCH)
    );
    
    // Test vector memory
    logic [31:0] instmem [0:255];    // instructions
    integer num_tests = 14;
    integer i;

    initial begin
    	R_EX=1'b1;
    	stall_EX=1'b0;
        // Load test vectors
        $readmemh("instmem.dat", instmem);
     	
	$display("Starting Control Unit Test Bench");
   	for (i = 0; i < num_tests; i++) begin
            instr = instmem[i];
            #1; // Wait for combinational logic to settle

                $display("X Mismatch at test %0d", i);
                $display("   Instr:    %b", instr);
         	$display("   aluop =   %b", aluop);
         	$display("   alusrc=   %b", alusrc);
         	$display("   regsel =  %b", regsel);
         	$display("   regwrite= %b", regwrite);
         	$display("   gpio_we = %b", gpio_we);
         	$display("   pcsrc = %b", pcsrc_EX);
         	$display("   stall_FETCH = %b", stall_FETCH);
        end

        $display("Control Unit Testing Complete");
    end
        logic clk;
    logic rst;
    // IO
    logic [31:0] gpio_in;
    logic [31:0] gpio_out;
    // DUT
    cpu my_cpu (
        .clk     (clk),
        .rst     (rst),       // cpu expects active-high reset
        .gpio_in (32'h12345678),
        .gpio_out(gpio_out)
    );
    // Clock generator (100 MHz-ish: 10 ns period)
    initial begin
        clk = 1'b0;
        forever begin
            #5 clk = ~clk;
        end
    end

    // Reset + stimulus
    initial begin
        // initialize signals
        gpio_in = 32'd12345;   // your test input
        rst = 1'b1;             // assert reset (active-high)
        #40;                    // hold reset for a few clock cycles
        rst = 1'b0;             // release reset -> CPU starts running

        #5000;                  // adjust as needed
        $display("GPIO out= %b", gpio_out);
        $display("SIM: finished.");
        $finish;
    end
endmodule
