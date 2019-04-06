/home/dormando/.apio/packages/toolchain-iverilog/bin/iverilog -B "/home/dormando/.apio/packages/toolchain-iverilog/lib/ivl" -o fifo_ram_tb.out -D VCD_OUTPUT=fifo_ram_tb "/home/dormando/.apio/packages/toolchain-iverilog/vlib/cells_sim.v" fifo_ram.v fifo_ram_tb.v simple_dual_ram.v
/home/dormando/.apio/packages/toolchain-iverilog/bin/vvp -M "/home/dormando/.apio/packages/toolchain-iverilog/lib/ivl" fifo_ram_tb.out

