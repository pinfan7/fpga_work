create_clock -name sys_clk -period 20.000 [get_ports {clk50M}]
# Only asynchronous external keys/reset are excluded; internal paths stay timed.
set_false_path -from [get_ports {reset_n key_enable_n key_minus_n key_center_n key_plus_n}]
# Output engineering budget; not a synchronous timing spec from the servo maker.
set_output_delay -clock sys_clk -max 5.000 [get_ports {servo_pwm_out led[*]}]
set_output_delay -clock sys_clk -min 0.000 [get_ports {servo_pwm_out led[*]}]
