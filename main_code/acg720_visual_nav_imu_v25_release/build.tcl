cd [file dirname [file normalize [info script]]]
open_project visual_nav_imu_v25.gprj
set_option -top_module ackermann_auto_top
set_option -verilog_std v2001
set_option -use_sspi_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -output_base_name visual_nav_imu_v25
run all
