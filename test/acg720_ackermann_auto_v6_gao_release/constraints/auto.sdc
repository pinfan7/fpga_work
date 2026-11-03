create_clock -name sys_clk -period 20.000 [get_ports {clk}]
create_generated_clock -name debug_clk -source [get_ports {clk}] -divide_by 1000000 [get_pins {probe_clk_s1/Q}]
# Only the vendor GAO JTAG clock is asynchronous; debug_clk is sys_clk-derived.
create_clock -name gao_tck -period 50.000 [get_pins {gw_gao_inst_0/tck_ibuf/I}]
set_clock_groups -asynchronous -group [get_clocks {sys_clk debug_clk}] -group [get_clocks {gao_tck}]
# Asynchronous inputs enter synchronizers. No broad clock group cuts.
set_false_path -from [get_ports {reset_n key_forward_n key_reverse_n ENC_LA ENC_LB ENC_RA ENC_RB imu_scl imu_sda}]
set_output_delay -clock sys_clk -max 5.000 [get_ports {PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
