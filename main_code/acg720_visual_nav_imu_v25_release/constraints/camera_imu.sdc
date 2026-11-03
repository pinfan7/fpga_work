# Physical oscillator is 50MHz. PLL/CLKDIV generated clocks are auto-derived.
create_clock -name clk50M -period 20.000 [get_ports {clk}]
# Conservative 100MHz camera input budget; actual sensor PCLK must be measured.
create_clock -name camera_pclk -period 10.000 [get_ports {camera_pclk}]
# No broad asynchronous clock-group cuts or blanket timing exceptions.
# Board-level input/output delays and vendor DDR CDC require hardware sign-off.
create_generated_clock -name debug_clk -source [get_ports {clk}] -divide_by 1000000 [get_pins {probe_clk_s1/Q}]
create_clock -name gao_tck -period 50.000 [get_pins {gw_gao_inst_0/tck_ibuf/I}]
# Only JTAG is unrelated to the fabric clocks. Do not cut motor/video domains.
set_clock_groups -asynchronous -group [get_clocks {gao_tck}] -group [get_clocks {clk50M debug_clk camera_pclk}]
set_false_path -from [get_ports {reset_n key_forward_n key_reverse_n ENC_LA ENC_LB ENC_RA ENC_RB}]
# UART inputs are asynchronous; each is sampled through uart_rx.sync[1:0].
set_false_path -from [get_ports {tof_l_rx tof_f_rx tof_r_rx}]
set_output_delay -clock clk50M -max 5.000 [get_ports {tof_l_tx tof_f_tx tof_r_tx}]
set_output_delay -clock clk50M -min 0.000 [get_ports {tof_l_tx tof_f_tx tof_r_tx}]
set_output_delay -clock clk50M -max 5.000 [get_ports {PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
set_output_delay -clock clk50M -min 0.000 [get_ports {PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
