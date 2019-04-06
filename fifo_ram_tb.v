`timescale 1ns / 1ps
`default_nettype none
`define DUMPSTR(x) `"x.vcd`"

module fifo_ram_tb(
    );

localparam ABITS = 11;
localparam DBITS = 8;

reg clk;
reg rst;

reg [DBITS-1:0] write_data;
reg write_en;

wire [DBITS-1:0] read_data;
reg read_en;

wire empty;
wire full;

fifo_ram #(.ADDRBITS(ABITS), .DATABITS(DBITS)) DUT (
    .clk(clk),
    .rst(rst),
    .write_data(write_data),
    .write_en(write_en),
    .read_data(read_data),
    .read_en(read_en),
    .empty(empty),
    .full(full)
);

// tick tock.
initial begin
    clk = 0;
    forever #10 clk = ~clk;
end

initial begin
    $dumpfile(`DUMPSTR(`VCD_OUTPUT));
    $dumpvars(0, fifo_ram_tb);

    rst = 1'b1;
    write_en = 1'b0;
    read_en = 1'b0;
    write_data = 8'h00;
    repeat(2) @(posedge clk);
    rst = 1'b0;
    repeat(2) @(posedge clk);

    // write write read write
    write_data = 8'h07;
    write_en = 1'b1;
    @(posedge clk);
    write_en = 1'b0;
    read_en = 1'b1;
    @(posedge clk);
    read_en = 1'b0;
    repeat(2) @(posedge clk);
    write_data = 8'hFA;
    write_en = 1'b1;
    @(posedge clk);
    write_en = 1'b0;
    read_en = 1'b1;
    @(posedge clk);
    read_en = 1'b0;
    write_data = 8'h1B;
    write_en = 1'b1;
    @(posedge clk);
    write_en = 1'b0;
    read_en = 1'b0;
    repeat(2) @(posedge clk);
    read_en = 1'b1;
    @(posedge clk)
    @(posedge clk)
    @(posedge clk)
    read_en = 1'b0;
    repeat(2) @(posedge clk);
    $finish;
end

endmodule
