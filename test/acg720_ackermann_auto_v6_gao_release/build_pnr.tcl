cd [file dirname [file normalize [info script]]]
open_project ackermann_auto_v6_gao.gprj
set_option -top_module ackermann_auto_top
set_option -use_sspi_as_gpio 1
set_option -output_base_name ackermann_auto_v6_gao
run pnr
