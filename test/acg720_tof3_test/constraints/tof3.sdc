create_clock -name sys_clk -period 20.000 [get_ports {clk}]
set_false_path -from [get_ports {reset_n key_forward_n key_left_n key_reverse_n key_right_n tof_l_rx tof_f_rx tof_r_rx}]
# Engineering budget for asynchronous UART and static test outputs.
set_output_delay -clock sys_clk -max 5.000 [get_ports {tof_l_tx tof_f_tx tof_r_tx debug_tx PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {tof_l_tx tof_f_tx tof_r_tx debug_tx PWMA AIN1 AIN2 PWMB BIN1 BIN2 STBY servo_pwm led[*]}]
