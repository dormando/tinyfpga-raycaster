#!/bin/bash
# icepll -m -f pll.v -i 16 -o 115
yosys -p "synth_ice40 -json top.json -blif hardware.blif" -q fifo_ram.v floordist_rom.v lcd_driver.v line_writer.v mmap_buffer.v mmap_interface.v mmap_protocol.v pll.v simple_dual_ram.v spi.v spi_slave.v task_crossdomain.v top.v && nextpnr-ice40 --lp8k --package cm81 --json ./top.json --pcf ./pins.pcf --asc hardware.asc --freq 16 && icepack hardware.asc hardware.bin
