// this central interface consumes the memory mapped protocol and controls the
// rest of the design, including rendering.
module mmap_interface (
    input clk,
    input rst,

    output busy,
    output reg lcd_wait,

    // Master interface
    input m_new_cmd,
    input m_write,
    input [5:0] m_cmd,
    input [31:0] m_address,
    input [31:0] m_data,

    // Slave interface
    output [31:0] s_data,
    output s_drdy,

    // line writer
    output reg [31:0] line_load,
    input line_write_ready,
    input line_write_done,
    output reg line_write_start,
    output [8:0] line_ram_raddr,
    input [15:0] line_ram_read_data,
    output reg [31:0] line_pos, // posx + posy
    // line writer texture memory.
    output reg [13:0] tram_waddr,
    output reg [3:0] tram_write_data,
    output reg tram_write_en,
    output reg [7:0] tpram_waddr,
    output reg [15:0] tpram_write_data,
    output reg tpram_write_en,

    // LCD driver
    output reg [7:0] lcd_data_in,
    output reg lcd_data_dcx, // low: data, hi: cmd
    output reg lcd_start,
    output lcd_rst,
    input lcd_done
);

// FIXME: reorg states and commands
localparam CMD_LEDS = 0,
    CMD_TEXP_LOAD = 1,
    CMD_TEX_LOAD = 3,
    CMD_CAST_LINE = 4,
    CMD_LCD = 5,
    CMD_SET_POS = 7;

reg [31:0] data_d, data_q;
reg drdy_d, drdy_q;

assign s_data = data_q;
assign s_drdy = drdy_q;
assign lcd_rst = lcd_rst_q;

localparam IDLE = 0,
    WAIT_LCD_DUMP = 3,
    WAIT_DUMP = 4,
    WAIT_LCD = 5,
    WAIT_FOR_CAST = 6,
    WAIT_FOR_LINE = 7,
    CAST_FB = 8,
    CAST_FB_START = 9;

localparam LCDIDLE = 0,
    LCD_FB_DUMP = 2,
    LCD_FB_READRAM_RES = 3,
    LCD_FB_SPLIT = 4,
    LCD_FB_SPLIT_WAIT = 5,
    LCD_FB_DUMP_START = 6;

reg [4:0] state_d, state_q;
reg [4:0] lcdstate_d, lcdstate_q;

reg [31:0] line_pos_d, line_pos_q; // posXY for line writer.
reg [8:0] lw_raddr_d, lw_raddr_q; // line writer
reg [8:0] lw_base_raddr_d, lw_base_raddr_q; // line RAM bank
assign line_ram_raddr = lw_raddr_q + lw_base_raddr_q;

// lcd
reg lcd_done_d, lcd_done_q; // FIXME: weird.
reg lcd_queue_d, lcd_queue_q; // track outstanding dump request.
reg lcd_rst_d, lcd_rst_q;
reg lcd_bcnt_d, lcd_bcnt_q;
reg [15:0] lcd_data_d, lcd_data_q;

// indicate to the arduino that it shouldn't send another cmd yet.
assign busy = state_q != IDLE;

initial begin
    lcd_rst_q = 1'b0;
end

always @(*) begin
        state_d = state_q;
        lcdstate_d = lcdstate_q;
        lcd_wait = 1'b0;
        data_d = data_q;
        drdy_d = 0;
        line_write_start = 1'b0;
        line_load = 32'd0;
        line_pos_d = line_pos_q;
        // line output register.
        // TODO: could remove most of line_writer's lines
        // if line's inputs were all just memory mapped addresses.
        line_pos = line_pos_q;
        lw_raddr_d = lw_raddr_q;
        lw_base_raddr_d = lw_base_raddr_q;

        lcd_done_d = lcd_done_q;
        lcd_queue_d = lcd_queue_q;

        // LCD regs
        lcd_data_in = 8'd0;
        lcd_data_dcx = 1'b1;
        lcd_start = 1'b0;
        lcd_rst_d = lcd_rst_q;
        lcd_bcnt_d = lcd_bcnt_q;
        lcd_data_d = lcd_data_q;

        // line texture/palette memory.
        tram_waddr = 13'd0;
        tram_write_data = 8'd0;
        tram_write_en = 1'b0;
        tpram_waddr = 13'd0;
        tpram_write_data = 8'd0;
        tpram_write_en = 1'b0;

        // TODO: refactor into a separate module. would need to route the
        // normal LCD commands through it as well. This currently lives here
        // because mmap_protocol is otherwise responsible for writing to the
        // LCD.
        case (lcdstate_q)
          LCDIDLE: begin
            /// whoooaa I forgot this for so long.
            lcd_done_d = 1'b0;
          end
          LCD_FB_DUMP: begin
            if (lw_raddr_q != 8'd240) begin
                lcdstate_d = LCD_FB_READRAM_RES;
            end else begin
                lcd_done_d = 1'b1;
                lcd_queue_d = 1'b0;
                lcdstate_d = LCDIDLE;
                // flip RAM line read bank.
                if (lw_base_raddr_q == 9'd0) begin
                    lw_base_raddr_d = 9'd240;
                end else begin
                    lw_base_raddr_d = 9'd0;
                end
            end
          end
          LCD_FB_READRAM_RES: begin
            lcd_data_d = line_ram_read_data;
            lcd_bcnt_d = 1'b0;
            lcdstate_d = LCD_FB_SPLIT;
          end
          LCD_FB_SPLIT: begin
            // for each byte, send each half to SPI.
            // TODO: experiment with a conditionally 16bit SPI driver to
            // simplify this logic a bit and speed things up.
            lcd_bcnt_d = ~lcd_bcnt_q;
            if (!lcd_bcnt_q) begin
                lcd_data_in = lcd_data_q[15:8];
            end else begin
                lcd_data_in = lcd_data_q[7:0];
            end

            lcd_start = 1'b1;
            lcdstate_d = LCD_FB_SPLIT_WAIT;
          end
          LCD_FB_SPLIT_WAIT: begin
            if (lcd_done) begin
                if (!lcd_bcnt_q) begin // if we've flipped back to zero.
                    lw_raddr_d = lw_raddr_q + 1'b1;
                    lcdstate_d = LCD_FB_DUMP;
                end else begin
                    lcdstate_d = LCD_FB_SPLIT;
                end
            end
          end
          default: lcdstate_d = LCDIDLE;
        endcase

        case (state_q)
          IDLE: begin
            if (m_new_cmd) begin
                case (m_cmd)
                    CMD_LCD: begin
                        // FIXME: making an assumption about readiness :P
                        // this causes race conditions and corrupts the LCD.
                        if (m_write) begin
                            state_d = WAIT_LCD;
                            lcd_data_in = m_data[7:0];
                            lcd_data_dcx = m_data[8];
                            lcd_start = 1'b1;
                        end else begin
                            // hack to allow LCD RST toggling.
                            lcd_rst_d = m_address[0];
                            drdy_d = 1'b1;
                        end
                    end
                    CMD_TEX_LOAD: begin
                        if (m_write) begin
                            tram_waddr = m_address[13:0];
                            tram_write_data = m_data[3:0];
                            tram_write_en = 1'b1;
                        end else begin
                            // no readback.
                            data_d = 32'd0;
                            drdy_d = 1'b1;
                        end
                    end
                    CMD_TEXP_LOAD: begin
                        if (m_write) begin
                            tpram_waddr = m_address[7:0];
                            tpram_write_data = m_data[15:0];
                            tpram_write_en = 1'b1;
                        end else begin
                            // no readback.
                            data_d = 32'd0;
                            drdy_d = 1'b1;
                        end
                    end
                    CMD_CAST_LINE: begin
                        if (m_write) begin
                            // we signal the line to "start" a number of times
                            // to read out initialization data.
                            // FIXME: probably better to pass an address
                            // and use a case on the line side? safer than
                            // lock-stepping states here.
                            // This is still generating a lot of wires as is.
                            line_load = m_data;
                            line_write_start = 1'b1;
                            if (m_address == 31'd4) begin
                                state_d = WAIT_FOR_LINE;
                            end
                        end else begin
                            data_d = 32'd0;
                            drdy_d = 1'b1;
                        end
                    end
                    CMD_SET_POS: begin
                        // TODO: not sure I like this being a separate
                        // command, but there's no reason not to yet.
                        if (m_write) begin
                            line_pos_d = m_data;
                        end else begin
                            data_d = 32'd0;
                            drdy_d = 1'b1;
                        end
                    end
                endcase
            end else begin
                //busy = 1'b0;
            end
          end
          WAIT_FOR_LINE: begin
              if (line_write_done) begin
                state_d = WAIT_LCD_DUMP;
              end
          end
          WAIT_LCD_DUMP: begin
            if (lcd_queue_q == 1'b1) begin
                // LCD still running from last request, have to wait.
                lcd_wait = 1'b1;
            end else begin
                // kick off LCD dump
                lw_raddr_d = 8'd0;
                // LCD is now queued.
                lcd_queue_d = 1'b1;
                lcdstate_d = LCD_FB_DUMP;
                // we can run another line write in the meantime.
                state_d = IDLE;
            end
          end
          WAIT_LCD: begin
            if (lcd_done) begin
                state_d = IDLE;
            end
          end
          default: state_d = IDLE;
        endcase
end

always @(posedge clk) begin
    if (rst) begin
        data_q <= 0;
        drdy_q <= 0;
    	state_q = 0;
        lcdstate_q = 0;
        lw_raddr_q <= 0;
        lw_base_raddr_q <= 0;
        lcd_bcnt_q <= 0;
        lcd_done_q <= 0;
        lcd_queue_q <= 0;
        lcd_data_q <= 0;
        line_pos_q <= 0;
      end else begin
        state_q = state_d;
        lcdstate_q = lcdstate_d;
        lcd_data_q <= lcd_data_d;
        data_q <= data_d;
        line_pos_q <= line_pos_d;
        drdy_q <= drdy_d;
        lw_raddr_q <= lw_raddr_d;
        lw_base_raddr_q <= lw_base_raddr_d;
        lcd_bcnt_q <= lcd_bcnt_d;
      end

      lcd_rst_q <= lcd_rst_d;
      lcd_done_q <= lcd_done_d;
      lcd_queue_q <= lcd_queue_d;
end

endmodule
