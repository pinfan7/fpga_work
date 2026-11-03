`timescale 1ns/1ps
// Boundary model replaces PLL/DDR/HDMI, not the motion controller. The real
// yellow detector and IMU session guard run inside it. Vendor elaboration is
// separately checked by the complete Gowin synthesis and place-and-route.
module gao_ov5640_ddr_hdmi #(parameter IMU_AUTO_START=0)(
 input clk50M,reset_n,imu_recal_n,source_key_n,osd_key_n,
 output reg signed [31:0] nav_yaw=0,
 output reg nav_imu_ok=0,output wire nav_imu_update,
 output hdmi_oe,motor_left_pwm,motor_left_in1,motor_left_in2,
 output motor_right_pwm,motor_right_in1,motor_right_in2,motor_stby,servo_safe,
 output rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,
 output [3:0] rgmii_txd,inout eth_mdio,imu_scl,imu_sda,
 output camera_sclk,camera_xclk,camera_rst_n,camera_pwdn,
 inout camera_sdat,input camera_vsync,camera_href,camera_pclk,
 input [7:0] camera_data,output [7:0] led,
 output tmds_clk_p,tmds_clk_n,output [2:0] tmds_data_p,tmds_data_n,
 output [13:0] O_ddr_addr,output [2:0] O_ddr_ba,
 output O_ddr_cs_n,O_ddr_ras_n,O_ddr_cas_n,O_ddr_we_n,
 output O_ddr_clk,O_ddr_clk_n,O_ddr_cke,O_ddr_odt,O_ddr_reset_n,
 output [1:0] O_ddr_dqm,inout [15:0] IO_ddr_dq,
 inout [1:0] IO_ddr_dqs,IO_ddr_dqs_n
);
 assign nav_imu_update=nav_imu_ok;
 reg freeze_pixels=0,video_fault=0,imu_fault=0;
 reg pixel_clock=0,imu_press=0;
 always @(posedge clk50M)if(!freeze_pixels)pixel_clock<=!pixel_clock;
 wire active,stopped;
 imu_session_guard #(.AUTO_START(IMU_AUTO_START)) session(
   .clk(clk50M),.rst_n(reset_n),.press(imu_press),.fault_pulse(imu_fault),
   .link_ok(!imu_fault),.configured(!imu_fault),.error_code(8'h26),
   .who_am_i(8'h47),.diagnostic(21'd0),.active(active),.stopped(stopped),
   .saved_error(),.saved_id(),.saved_diagnostic());
 reg [23:0] rgb=0;reg de=0,vs=0;
 wire box_valid;wire [11:0] x0,y0,x1,y1;
 yellow_cup_detector #(.WIDTH(64),.HEIGHT(48),.MIN_TILES(4),.MIN_W(2),.MIN_H(2)) detect(
   .clk(pixel_clock),.rst_n(reset_n && !video_fault),.enable(1'b1),
   .rgb(rgb),.de(de),.vs(vs),.box_valid(box_valid),
   .box_x0(x0),.box_y0(y0),.box_x1(x1),.box_y1(y1));
 assign led={video_fault,imu_fault,active,5'b0};
 assign hdmi_oe=reset_n && !video_fault;
 // Original vision safety outputs are intentionally ignored by the real top.
 assign {motor_left_pwm,motor_left_in1,motor_left_in2,motor_right_pwm,
         motor_right_in1,motor_right_in2,motor_stby,servo_safe}=0;
 task send_frame;
 integer x,y;
 begin
   @(negedge pixel_clock);vs=1;de=0;
   repeat(4)@(negedge pixel_clock);vs=0;
   for(y=0;y<48;y=y+1)begin
     for(x=0;x<64;x=x+1)begin
       @(negedge pixel_clock);de=1;
       rgb=(x>=16 && x<40 && y>=8 && y<40) ? 24'hdcc820 : 24'h202020;
     end
     @(negedge pixel_clock);de=0;
     repeat(4)@(negedge pixel_clock);
   end
   repeat(4000)@(negedge pixel_clock);
 end
 endtask
endmodule

