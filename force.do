force -freeze sim:/cpu/clk 1 0, 0 {50 ns} -r 100
force -freeze sim:/cpu/rst 1'h1 0
force -freeze sim:/cpu/gpio_in 32'h4 0
run 1000
force -freeze sim:/cpu/rst 1'h0 0
run 10000
