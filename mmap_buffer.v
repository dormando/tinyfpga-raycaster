module mmap_buffer
(
    input clk,
    input rst,

    input [7:0] rx_data,
    input new_rx_data,

    input mp_busy,
    output reg [7:0] mp_rx_data,
    output reg mp_new_rx_data,

    output reg overflow,
    output clear,
    output mb_full
);

    reg [7:0] write_data;
    reg write_en;

    wire [7:0] read_data;
    reg read_en;

    wire empty;
    wire full;
    assign mb_full = full;
    assign clear = empty;

    fifo_ram #(.ADDRBITS(13), .DATABITS(8)) fifo_ram (
        .clk(clk),
        .rst(rst),
        .write_data(write_data),
        .write_en(write_en),
        .read_data(read_data),
        .read_en(read_en),
        .empty(empty),
        .full(full)
    ); 
  
    localparam IDLE = 0,
        READ = 1; 
    reg [1:0] state_d, state_q;

    always @(*) begin
        state_d = state_q;
        read_en = 1'b0;
        write_en = 1'b0;
        write_data = 0;
        mp_new_rx_data = 1'b0;
        mp_rx_data = 0;
        overflow = 1'b0;

        if (new_rx_data) begin
            if (full) begin
                overflow = 1'b1;
            end else begin
                write_data = rx_data;
                write_en = 1'b1;
            end
        end

        case (state_q)
            IDLE: begin
                if (!mp_busy && !empty) begin
                    // reads become available on the next cycle.
                    read_en = 1'b1;
                    state_d = READ;
                end
            end
            READ: begin
                mp_rx_data = read_data;
                mp_new_rx_data = 1'b1;
                state_d = IDLE;
            end
        endcase
    end

    always @(posedge clk) begin
        if (rst) begin
            state_q = IDLE;
        end else begin
            state_q = state_d;
        end
    end

endmodule
