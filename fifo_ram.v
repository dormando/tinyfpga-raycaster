module fifo_ram #(
    parameter ADDRBITS = 11,
    parameter DATABITS = 8
)(
    // write interface
    input clk,
    input rst,
    input [DATABITS-1:0] write_data,
    input write_en,

    // read interface
    output [DATABITS-1:0] read_data,
    input read_en, // need to explicitly signal reads to move ptrs

    // signal bits
    output empty,
    output full
);
    localparam READOP = 2'b01;
    localparam WRITEOP = 2'b10;
    localparam READWRITEOP = 2'b11;

    //reg [DATABITS-1:0] mem [ADDRBITS-1:0];

    reg [ADDRBITS-1:0] wr_ptr_d, wr_ptr_q, rd_ptr_d, rd_ptr_q;
    reg [ADDRBITS-1:0] fill_d, fill_q;
    reg empty_d, empty_q, full_d, full_q;

    reg [ADDRBITS-1:0] ram_raddr;
    reg [ADDRBITS-1:0] ram_waddr;
    reg ram_write_en;

    // TODO: figure out if using mem directly above would use BRAM or if we
    // still have to use this external module.
    // FIXME: Need to use a macro to specify DEPTH. Was getting warnings.
    simple_dual_ram #(.SIZE(DATABITS), .DEPTH(8192)) ram (
        .wclk(clk),
        .waddr(ram_waddr),
        .write_data(write_data),
        .write_en(ram_write_en),
        // read
        .rclk(clk),
        .raddr(ram_raddr),
        .read_data(read_data)
    );

    assign empty = empty_q;
    assign full  = full_q;

    // TODO: I don't know if the + 1'b1 in two places takes up more logic than
    // pre calculating the value into a register first.
    always @(*) begin
        wr_ptr_d = wr_ptr_q;
        rd_ptr_d = rd_ptr_q;
        empty_d = empty_q;
        full_d = full_q;
        fill_d = fill_q;
        ram_raddr = rd_ptr_q;
        ram_waddr = 0;
        ram_write_en = 1'b0;

        case ({write_en, read_en}) 
            READOP: begin
                // READ: if not empty, allow read.
                // if read, can't be full next cycle.
                if (!empty_q) begin
                   // successfully advance read pointer.
                   rd_ptr_d = rd_ptr_q + 1'b1;
                   full_d = 1'b0; 

                   fill_d = fill_d - 1'b1;
                   if (fill_q == 1'b1) begin
                     empty_d = 1'b1;
                   end
                end
            end
            
            // WRITE: if not full, allow write
            // if write, can't be empty next cycle.
            // might be full next cycle.
            WRITEOP: begin
                if (!full_q) begin
                    // successfully advance write pointer
                    wr_ptr_d = wr_ptr_q + 1'b1;
                    empty_d = 1'b0;
                    ram_waddr = wr_ptr_q;
                    ram_write_en = 1'b1;

                    fill_d = fill_d + 1'b1;
                    if (&fill_d) begin
                      full_d = 1'b1;
                    end
                end
            end

            // READ and WRITE: can't be empty or full
            // FIXME: rd/wr same addr?
            READWRITEOP: begin
                rd_ptr_d = rd_ptr_q + 1'b1;
                wr_ptr_d = wr_ptr_q + 1'b1;
                ram_waddr = wr_ptr_q;
                ram_write_en = 1'b1;
            end
            default: begin
                // nothing.
            end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr_q <= 0;
            rd_ptr_q <= 0;
            full_q <= 0;
            empty_q <= 1'b1;
            fill_q <= 0;
        end else begin
            wr_ptr_q <= wr_ptr_d;
            rd_ptr_q <= rd_ptr_d;
            full_q <= full_d;
            empty_q <= empty_d;
            fill_q <= fill_d;
        end
    end

endmodule
