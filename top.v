// look in pins.pcf for all the pin names on the TinyFPGA BX board
module top (
    input CLK,     // 16MHz clock
    output LED,    // User/boot LED next to power LED
    output USBPU,  // USB pull-up resistor
    input PIN_10,  // arduino CS
    input PIN_11,  // arduino MOSI
    output PIN_12, // arduino MISO
    input PIN_13,  // arduino SPI clock (SCK)
    output LCD_DCX, // hi: data, lo: cmd
    output LCD_RST, // lcd seems to hang without RST
    output LCD_SS,
    output LCD_MOSI,
    output LCD_SCK,
    input PIN_8,  // RST signal from arduino
    output PIN_9 // indicate busy to arduino

);
    // drive USB pull-up resistor to '0' to disable USB
    assign USBPU = 0;
    reg led_enabled = 0;
    wire mmap_busy;
    // We let the arduino know when our mmap buffer input queue is clear. This
    // signal should be combined with an indicator that the mmap protocol and
    // LCD writer are all idle.
    assign PIN_9 = mb_clear;

    // Use a blank register and hold reset for a while on first startup
    // taken from the picosoc hardware.v example.
    // Also: the arduino can issue an RST on its own. Uses a debouncer since
    // RST could be triggered by simply turning the arduino on.
    reg [5:0] reset_cnt = 0;
    reg rstc = !(&reset_cnt);
    reg [10:0] rst_debounce = 0;
    reg rst_wire = 0;
    always @(posedge CLK) begin
        rst_wire <= 1'b0;
        reset_cnt <= reset_cnt + rstc;
        if (PIN_8) begin
            if (!(&rst_debounce)) begin
                rst_debounce <= rst_debounce + 1'b1;
            end else begin
                rst_wire <= 1'b1;
            end
        end else begin
            rst_debounce <= 0;
        end
    end
    wire RST;
    assign RST = rst_wire ? 1'b1 : rstc;

    // fast clock for LCD SPI
    wire FCLK, FLOCKED;
    pll pll (
        .clock_in(CLK),
        .clock_out(FCLK),
        .locked(FLOCKED)
    );

    // SPI from arduino
    wire spi_done;
    wire [7:0] spi_dout;

    // TODO: Nothing to send back to arduino yet.
    spi_slave spi_slave (
        .clk(CLK),
        .rst(RST),
        .ss(PIN_10),
        .mosi(PIN_11),
        .miso(PIN_12), // no output yet.
        .sck(PIN_13),
        .done(spi_done),
        .din(8'hff), // same, no output.
        .dout(spi_dout)
    );

    // LCD module wires
    wire [7:0] lcd_data_in;
    wire lcd_data_dcx;
    wire lcd_start;
    wire lcd_busy;
    wire lcd_done;
    wire lcd_rst;
    assign LCD_RST = lcd_rst;

    lcd_driver lcd_driver (
        .clk(CLK),
        .fclk(FCLK),
        .rst(RST),
        .lcd_sck(LCD_SCK),
        .lcd_mosi(LCD_MOSI),
        .lcd_ss(LCD_SS),
        .lcd_dcx(LCD_DCX),
        .data_in(lcd_data_in),
        .data_dcx(lcd_data_dcx),
        .start(lcd_start),
        .busy(lcd_busy),
        .done(lcd_done)
    );

    // Memory mapping protocol wires.
    wire m_new_cmd;
    wire m_write;
    wire [5:0] m_cmd;
    wire [31:0] m_address;
    wire [31:0] m_data;
    wire [31:0] s_data;
    wire s_drdy;
    wire mp_busy;

    // REQUEST_WRITE, block on mmap_busy
    // (also REQUEST_READ, but it's unused)
    // module sits between spi_done/dout and mmap_protocol
    // if mmap_protocol is not busy, read bytes until fifo empty or busy
    // again.
   
    wire [7:0] mp_rx_data;
    wire mp_new_rx_data;
    wire mb_overflow;
    wire mb_clear;
    wire mb_full;
    mmap_buffer mmap_buffer (
        .clk(CLK),
        .rst(RST),
        // lines from SPI.
        .rx_data(spi_dout),
        .new_rx_data(spi_done),
        // lines to mmap_protocol
        .mp_busy(mp_busy),
        .mp_rx_data(mp_rx_data),
        .mp_new_rx_data(mp_new_rx_data),
        .overflow(mb_overflow),
        .clear(mb_clear),
        .mb_full(mb_full)
    );

    mmap_protocol mmap_protocol (
        .clk(CLK),
        .rst(RST),
        .rx_data(mp_rx_data),
        .new_rx_data(mp_new_rx_data),
        // tx_data, new_tx_data
        .tx_busy(1'b0),
        .m_new_cmd(m_new_cmd),
        .m_write(m_write),
        .m_cmd(m_cmd),
        .m_address(m_address),
        .m_data(m_data),
        .m_busy(mmap_busy),
        .s_data(s_data),
        .s_drdy(s_drdy),
        .busy(mp_busy)
    );

    // line writer wires
    wire [31:0] line_load;
    wire [31:0] line_pos;
    wire line_write_start;
    wire line_write_done;
    wire line_write_ready;
    wire [8:0] line_ram_raddr;
    wire [15:0] line_ram_read_data;
    wire [13:0] line_tram_waddr;
    wire [3:0] line_tram_write_data;
    wire line_tram_write_en;
    wire [7:0] line_tpram_waddr;
    wire [15:0] line_tpram_write_data;
    wire line_tpram_write_en;

    wire lcd_wait;
    mmap_interface mmap_interface (
        .clk(CLK),
        .rst(RST),
        .busy(mmap_busy),
        .lcd_wait(lcd_wait), // waiting on the LCD to flush before we can start another line. currently unused
        .m_new_cmd(m_new_cmd),
        .m_write(m_write),
        .m_cmd(m_cmd),
        .m_address(m_address),
        .m_data(m_data),
        .s_data(s_data),
        .s_drdy(s_drdy),
        .line_load(line_load),
        .line_pos(line_pos),
        .line_write_ready(line_write_ready),
        .line_write_done(line_write_done),
        .line_write_start(line_write_start),
        .line_ram_raddr(line_ram_raddr),
        .line_ram_read_data(line_ram_read_data),
        .tram_waddr(line_tram_waddr),
        .tram_write_data(line_tram_write_data),
        .tram_write_en(line_tram_write_en),
        .tpram_waddr(line_tpram_waddr),
        .tpram_write_data(line_tpram_write_data),
        .tpram_write_en(line_tpram_write_en),
        // LCD driver
        .lcd_data_in(lcd_data_in),
        .lcd_data_dcx(lcd_data_dcx),
        .lcd_start(lcd_start),
        .lcd_rst(lcd_rst),
        .lcd_done(lcd_done)
    );

    line_writer line_writer (
        .clk(CLK),
        .rst(RST),
        .start(line_write_start),
        .load(line_load),
        .pos(line_pos),
        .rdy(line_write_ready),
        .done(line_write_done),
        .ram_raddr(line_ram_raddr),
        .ram_read_data(line_ram_read_data),
        .tram_waddr(line_tram_waddr),
        .tram_write_data(line_tram_write_data),
        .tram_write_en(line_tram_write_en),
        .tpram_waddr(line_tpram_waddr),
        .tpram_write_data(line_tpram_write_data),
        .tpram_write_en(line_tpram_write_en)
    );

    assign LED = led_enabled;
    reg [18:0] blink_counter = 0;
    always @(posedge CLK) begin
       blink_counter <= blink_counter + 1;
       // Just have the one LED with the TinyFPGA BX :)
       // currently detecting overflows in the mmap buffer.
       if (mb_overflow) begin
           led_enabled <= 1'b1;
           blink_counter <= 0;
        end else begin
           //led_enabled <= 1'b0;
           if (blink_counter[18]) begin
               led_enabled <= 1'b0;
           end
       end
    end
endmodule
