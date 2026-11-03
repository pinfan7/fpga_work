# Physical oscillator is 50MHz. PLL/CLKDIV generated clocks are auto-derived.
create_clock -name clk50M -period 20.000 [get_ports {clk50M}]
# Conservative 100MHz camera input budget; actual sensor PCLK must be measured.
create_clock -name camera_pclk -period 10.000 [get_ports {camera_pclk}]
# No broad asynchronous clock-group cuts or blanket timing exceptions.
# Board-level input/output delays and vendor DDR CDC require hardware sign-off.
