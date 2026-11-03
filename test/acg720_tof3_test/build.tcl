cd [file dirname [info script]]
open_project tof3_test.gprj
set_option -top_module tof3_test_top
set_option -verilog_std v2001
set_option -use_sspi_as_gpio 1
set_option -output_base_name tof3_test
if {[llength [info commands save_project]]} { save_project }
run all
