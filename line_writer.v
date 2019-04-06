// This module writes lines into a memory buffer, later to be blit to the
// screen.
// It was originally very simple; floors, ceilings, and sprites complicate it
// significantly.
// A few speedups and simplifications are possible:
// 1) mirror floor and ceiling writes (split the RAM into two modules for top
// and bottom half)
// 2) skip floor/ceiling calculations when drawing the wall :P
// 3) move some of the sprite logic (ie; x calculation) out into
// sprite_scanline.
module line_writer (
    input clk,
    input rst,

    input start,
    input [31:0] load, // generic loader wires.
    input [31:0] pos, // posx + posy inputs

    output reg rdy,
    output reg done,

    // external LCD write reads from memory.
    input [RAM_ASIZE:0] ram_raddr,
    output [15:0] ram_read_data,

    // external mmap interface loads texture memory.
    input [13:0] tram_waddr,
    input [3:0] tram_write_data,
    input tram_write_en,
    // external mmap interface also loads texture palettes.
    input [7:0] tpram_waddr,
    input [15:0] tpram_write_data,
    input tpram_write_en,
);

// TODO: Should be a configuration wire.
localparam HALF_HEIGHT = 8'd120;
localparam HEIGHT_MAX = 8'd239;
localparam RAM_ASIZE = 8;
localparam TEX_WIDTH = 32;
localparam TEX_SIZE = 11'd1024; // 32 * 32
localparam PAL_SIZE = 16;
localparam FLOOR_OFFSET = 6144;
localparam CEILING_OFFSET = 7168;
localparam FLOOR_PAL = 96;
localparam CEILING_PAL = 112;

reg [3:0] state_d, state_q;
reg [7:0] count_d, count_q; // number of pixels filled
reg [7:0] texid_d, texid_q;
reg [6:0] tex_x_d, tex_x_q;
reg [7:0] tex_initial_d, tex_initial_q;
reg [15:0] floor_x_d, floor_x_q;
reg [15:0] floor_y_d, floor_y_q;
reg [15:0] pos_x_d, pos_x_q;
reg [15:0] pos_y_d, pos_y_q;

// TODO: these lines are at most 8 bits (240 height)
// there's no reason to not also pre-calc the start/end values into the line
// height table, since the line height is never used.
reg [7:0] draw_start_d, draw_start_q;
reg [7:0] draw_end_d, draw_end_q;
reg [15:0] scale_d, scale_q;
reg [15:0] distwall_d, distwall_q;
reg [21:0] tex_offset_d, tex_offset_q; // Q13.8 -> 20:8 used for TRAM index.
reg [7:0] texp_offset_d, texp_offset_q;
reg [15:0] weight_d, weight_q;
reg [15:0] cfloor_x_d, cfloor_x_q;
reg [15:0] cfloor_y_d, cfloor_y_q;

// line RAM
reg [RAM_ASIZE:0] waddr_base_d, waddr_base_q;
reg [RAM_ASIZE:0] ram_waddr;
reg [15:0] ram_write_data;
reg ram_write_en;

// texture RAM
reg [13:0] tram_holdr_d, tram_holdr_q; // extra indirection for sprite writer.
wire [3:0] tram_read_data;
reg [13:0] tram_raddr;
wire [15:0] tpram_read_data;
reg [7:0] tpram_raddr;

simple_dual_ram #(.SIZE(16), .DEPTH((HEIGHT_MAX+1)*2)) line_ram (
    .wclk(clk),
    .waddr(ram_waddr),
    .write_data(ram_write_data),
    .write_en(ram_write_en),
    // read
    .rclk(clk),
    .raddr(ram_raddr),
    .read_data(ram_read_data)
);

// texture RAM module. holds 32x32 textures with up to 4 bit color.
// would be one fewer wire if mmap owned it, but harder to test bench.
// TODO: simple_dual_ram has equal sized read/write ports. mmap interface can
// write four bytes at once, but to avoid excess logic we're only loading
// one per command.

simple_dual_ram #(.SIZE(4), .DEPTH(10240)) texture_ram (
    .wclk(clk),
    .waddr(tram_waddr),
    .write_data(tram_write_data),
    .write_en(tram_write_en),
    // read
    .rclk(clk),
    .raddr(tram_raddr),
    .read_data(tram_read_data)
);

simple_dual_ram #(.SIZE(16), .DEPTH(256)) texture_pal_ram (
    .wclk(clk),
    .waddr(tpram_waddr),
    .write_data(tpram_write_data),
    .write_en(tpram_write_en),
    // read
    .rclk(clk),
    .raddr(tpram_raddr),
    .read_data(tpram_read_data)
);

// floor dist precalcs
reg [8:0] fd_addr;
wire [15:0] floordist;
floordist_rom floordist_rom (
    .clk(clk),
    .addr(fd_addr),
    .floordist(floordist)
);

localparam IDLE = 0,
    FILLING = 1,
    FILL_PRECALC = 2,
    FILL_PRECALC2 = 3,
    FILL_PRECALC3 = 4,
    CHECK_PAL = 5,
    FINISH = 6,
    LOAD_A = 7,
    LOAD_B = 8,
    LOAD_C = 9,
    LOAD_D = 10,
    LOAD_E = 11;

initial begin
    draw_end_q = 16'd0;
    draw_start_q = 16'd0;
    texid_q = 8'd0;
    count_q = 8'd0;
end

always @(*) begin
    state_d = state_q;
    count_d = count_q;
    draw_start_d = draw_start_q;
    draw_end_d = draw_end_q;
    scale_d = scale_q;
    tex_offset_d = tex_offset_q;
    waddr_base_d = waddr_base_q;
    ram_waddr = count_q + waddr_base_q;
    ram_write_en = 1'b0;
    ram_write_data = 8'd0;
    rdy = 0;
    done = 0;
    texid_d = texid_q;
    tex_x_d = tex_x_q;
    tex_initial_d = tex_initial_q;
    tram_raddr = tex_offset_q[21:8];
    tpram_raddr = 0;
    fd_addr = count_q;
    floor_x_d = floor_x_q;
    floor_y_d = floor_y_q;
    pos_x_d = pos_x_q;
    pos_y_d = pos_y_q;
    distwall_d = distwall_q;
    weight_d = weight_q;
    cfloor_x_d = cfloor_x_q;
    cfloor_y_d = cfloor_y_q;
    texp_offset_d = texp_offset_q;
    tram_holdr_d = tram_holdr_q;

    case (state_q)
        IDLE: begin
            if (start) begin
                state_d = LOAD_A;
                count_d = 0;
                draw_start_d = load[31:16];
                draw_end_d = load[15:0];
                pos_x_d = pos[31:16];
                pos_y_d = pos[15:0];
            end else begin
                rdy = 1'b1;
            end
        end
        LOAD_A: begin
            if (start) begin
                state_d = LOAD_B;
                scale_d = load[31:16];
                // FIXME: set up tex_initial_d.
                tex_initial_d = load[7:0];
            end
        end
        LOAD_B: begin
            if (start) begin
                state_d = LOAD_C;
                tex_x_d = load[22:16];
                texid_d = load[7:0];
            end
        end
        LOAD_C: begin
            // floorx/floory
            if (start) begin
                state_d = LOAD_D;
                floor_x_d = load[31:16];
                floor_y_d = load[15:0];
            end
        end
        LOAD_D: begin
            // inverse distance
            if (start) begin
                state_d = FILL_PRECALC;
                distwall_d = load[15:0];
                // done preloading, do kickoff.
                // FIXME: defines for magic constants.
                tex_offset_d[21:8] = ((texid_q[3:0]) << 10) + (tex_x_q << 5) + tex_initial_q;
                tex_offset_d[7:0] = 8'd0;
                texp_offset_d = ((texid_q[3:0]) << 4);
            end
        end
        FILL_PRECALC: begin
            weight_d = trunc_fixed_mul(floordist * distwall_q);
            state_d = FILL_PRECALC2;
        end
        FILL_PRECALC2: begin
            // bits 7:3 are the final texture coordinates
            cfloor_x_d = trunc_fixed_mul(weight_q * floor_x_q) + trunc_fixed_mul((16'h01_00 - weight_q) * pos_x_q);
            cfloor_y_d = trunc_fixed_mul(weight_q * floor_y_q) + trunc_fixed_mul((16'h01_00 - weight_q) * pos_y_q);
            state_d = FILL_PRECALC3;
        end
        FILL_PRECALC3: begin
            // optimization of a cfloor * texWidth|Height % texWidth|Height.
            // << 5 (texwidth). cfloor is now a multiple of tex size
            // the modulus is thus the 5 bits to the right of the . in Q8.8
		    // FIXME: Can remove this cycle if adding a bit more to previous
            // cycles and filling tram_raddr above.
            if (count_q < draw_start_q) begin
                tram_raddr = CEILING_OFFSET + (cfloor_x_q[7:3] << 5) + cfloor_y_q[7:3];
            end else if (count_q > draw_end_q) begin
                // override the texture RAM lookup if we're casting to the
                // floor.
                tram_raddr = FLOOR_OFFSET + (cfloor_x_q[7:3] << 5) + cfloor_y_q[7:3];
            end
            state_d = CHECK_PAL;
        end
        CHECK_PAL: begin
            tpram_raddr = texp_offset_q + tram_read_data;
            if (count_q < draw_start_q) begin
                tpram_raddr = CEILING_PAL + tram_read_data;
            end else if (count_q > draw_end_q) begin
                tpram_raddr = FLOOR_PAL + tram_read_data;
            end
            state_d = FILLING;
        end
        FILLING: begin
            count_d = count_q + 1'b1;
            if (count_q < draw_start_q) begin
                // ceiling gets normal color.
                ram_write_data = tpram_read_data;
            end else if (count_q >= draw_start_q && count_q <= draw_end_q) begin
                // if "side", darken the color
                // RGB 565
                // FIXME: function.
                if (texid_q[7]) begin
                    ram_write_data[15:11] = tpram_read_data[15:11] >> 1;
                    ram_write_data[10:5] = tpram_read_data[10:5] >> 1;
                    ram_write_data[4:0] = tpram_read_data[4:0] >> 1;
                end else begin
                    ram_write_data = tpram_read_data;
                end
                tex_offset_d = tex_offset_q + scale_q;
            end else if (count_q > draw_end_q) begin
                // floor, darken color.
                ram_write_data[15:11] = tpram_read_data[15:11] >> 1;
                ram_write_data[10:5] = tpram_read_data[10:5] >> 1;
                ram_write_data[4:0] = tpram_read_data[4:0] >> 1;
            end
            ram_write_en = 1'b1;

            if (count_q == 239) begin
                state_d = FINISH;
            end else begin
                count_d = count_q + 1'b1;
                state_d = FILL_PRECALC;
            end
        end
        FINISH: begin
            done = 1'b1;
            if (waddr_base_q == 9'd0) begin
                waddr_base_d = 9'd240;
            end else begin
                waddr_base_d = 9'd0;
            end
            state_d = IDLE;
        end
    endcase
end

always @(posedge clk) begin
    if (rst) begin
        state_q = IDLE;
        count_q <= 0;
        draw_start_q <= 0;
        draw_end_q <= 0;
        texid_q <= 0;
        tex_offset_q <= 0;
        waddr_base_q <= 0;
    end else begin
        state_q = state_d;
        count_q <= count_d;
        draw_start_q <= draw_start_d;
        draw_end_q <= draw_end_d;
        texid_q <= texid_d;
        tex_offset_q <= tex_offset_d;
        waddr_base_q <= waddr_base_d;
    end

    tex_x_q <= tex_x_d;
    tex_initial_q <= tex_initial_d;
    scale_q <= scale_d;
    floor_x_q <= floor_x_d;
    floor_y_q <= floor_y_d;
    pos_x_q <= pos_x_d;
    pos_y_q <= pos_y_d;
    distwall_q <= distwall_d;
    weight_q <= weight_d;
    cfloor_x_q <= cfloor_x_d;
    cfloor_y_q <= cfloor_y_d;
    texp_offset_q <= texp_offset_d;
end

function signed [15:0] trunc_fixed_mul(input signed [31:0] mulres);
  trunc_fixed_mul = mulres[23:8];
endfunction

function signed [31:0] trunc_fixed_mul32(input signed [63:0] mulres);
  trunc_fixed_mul32 = mulres[47:16];
endfunction

endmodule
