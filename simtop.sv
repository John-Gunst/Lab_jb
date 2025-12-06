//framos jgunst
module simtop;

    reg clk;
    reg rst_n;
    reg [31:0] tb_switches;
    wire tb_gpio_we;
    wire [31:0] tb_gpio_wdata;

    // Instantiate cpu (matches your cpu header)
    cpu uut (
        .clk        (clk),
        .rst_n      (rst_n),
        .io0_in     (tb_switches),
        .gpio_we    (tb_gpio_we),
        .gpio_wdata (tb_gpio_wdata)
    );

    // Clock generation (100 MHz)
    initial clk = 0;
    always #5 clk = ~clk;

    // VCD dump for waveform visualization
    initial begin
        $dumpfile("simtop.vcd");
        $dumpvars(0, uut);
    end

    // Helper task for running test cases
    task run_case(input [31:0] switches, input integer cycles, input string label);
        begin
            tb_switches = switches;
            $display("\n--- Test: %s | switches = %0d (0x%0h) ---", label, switches, switches);
            repeat (cycles) @(posedge clk);

            // Wait for gpio_we to be set
            wait (tb_gpio_we == 1'b1);

            $display("  gpio_we=%0b gpio_wdata=0x%08h (%0d)",
                     tb_gpio_we, tb_gpio_wdata, tb_gpio_wdata);
        end
    endtask

    // Main stimulus
    initial begin
        $display("Starting simulation...");

        rst_n = 1'b0;
        tb_switches = 32'd0;
        repeat(10) @(posedge clk);  // wait for reset
        rst_n = 1'b1;  // release reset

        // Run tests with test cases
        run_case(32'd2, 100, "sqrt(2)");
        run_case(32'd17, 100, "sqrt(17)");
        run_case(32'd12345, 100, "sqrt(12345)");

        // Check gpio_wdata and gpio_we after all tests
        $display("After 100 cycles: gpio_we=%b gpio_wdata=0x%08h", tb_gpio_we, tb_gpio_wdata);

        $display("Simulation finished.");
        $finish;
    end
endmodule

