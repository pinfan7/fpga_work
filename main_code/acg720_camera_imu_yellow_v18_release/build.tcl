cd [file dirname [file normalize [info script]]]
open_project camera_imu_yellow_v18.gprj
set_option -top_module gao_ov5640_ddr_hdmi
set_option -verilog_std v2001
set_option -use_sspi_as_gpio 1
set_option -output_base_name camera_imu_yellow_v18
run all
