cd [file dirname [file normalize [info script]]]
open_project ackermann_servo_v2.gprj
set_option -top_module acg720_servo_top
set_option -verilog_std v2001
set_option -use_sspi_as_gpio 1
set_option -output_base_name ackermann_servo_v2
run all
