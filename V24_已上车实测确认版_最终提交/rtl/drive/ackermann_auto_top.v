`timescale 1ns/1ps
module ackermann_auto_top #(
 parameter integer CLK_HZ=50000000, ENC_CPR=1560,WHEEL_UM=65000,
 parameter integer SERVO_CENTER_US=1400,SERVO_LEFT_US=1039,SERVO_RIGHT_US=1850,
 parameter integer GOAL_MM=2000,YAW_SIGN=1,
 parameter integer CAR_WIDTH_MM=200,CAR_LENGTH_MM=270,
 parameter integer OBSTACLE_WIDTH_MM=88,OBSTACLE_LENGTH_MM=88,CLEARANCE_MM=60,
 parameter integer SENSOR_FORWARD_MM=270,
 parameter integer LEFT_TRIM_PERCENT=105,RIGHT_TRIM_PERCENT=100,
 parameter integer TOF_BAUD=9600,TOF_TIMEOUT_MS=300,
 parameter integer TOF_INIT_DELAY_MS=1000,TOF_COMMAND_WAIT_MS=150,
 parameter integer AVOID_TRIGGER_MM=400,AVOID_RPM=20,MAX_RPM=80,START_WAIT_MS=1000,
 parameter integer LOOK_GAIN_Q8=128,HEADING_KP_US=64,
 parameter integer MOTOR_L_REVERSE=0,MOTOR_R_REVERSE=0,ENC_L_REVERSE=0,ENC_R_REVERSE=0
)(
 input clk,reset_n,key_forward_n,key_reverse_n,
 input ENC_LA,ENC_LB,ENC_RA,ENC_RB,
 input tof_l_rx,tof_f_rx,tof_r_rx,
 output tof_l_tx,tof_f_tx,tof_r_tx,
 output PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,servo_pwm,
 inout imu_scl,imu_sda,output [7:0] led,
 // Independent vision peripherals: S2 source, S4 IMU session; S3 is STOP only.
 input source_key_n,imu_recal_n,
 output hdmi_oe,
 output camera_sclk,camera_xclk,camera_rst_n,camera_pwdn,
 inout camera_sdat,
 input camera_vsync,camera_href,camera_pclk,
 input [7:0] camera_data,
 output tmds_clk_p,tmds_clk_n,
 output [2:0] tmds_data_p,tmds_data_n,
 output [13:0] O_ddr_addr,
 output [2:0] O_ddr_ba,
 output O_ddr_cs_n,O_ddr_ras_n,O_ddr_cas_n,O_ddr_we_n,
 output O_ddr_clk,O_ddr_clk_n,O_ddr_cke,O_ddr_odt,O_ddr_reset_n,
 output [1:0] O_ddr_dqm,
 inout [15:0] IO_ddr_dq,
 inout [1:0] IO_ddr_dqs,IO_ddr_dqs_n,
 output rgmii_gtxc,rgmii_txen,eth_mdc,eth_rst_n,
 output [3:0] rgmii_txd,
 inout eth_mdio
);
 // IMU heading is exported in this same 50MHz oscillator domain.
 // Video/DDR completion does not gate navigation; IMU validity does.
 wire signed [31:0] nav_yaw;
 wire nav_imu_ok,nav_imu_update;
 reg [31:0] imu_age_clks;
 wire imu_fresh=nav_imu_ok && imu_age_clks<CLK_HZ/4;
 always @(posedge clk or negedge reset_n)
   if(!reset_n)imu_age_clks<=CLK_HZ/4;
   else if(nav_imu_update && nav_imu_ok)imu_age_clks<=0;
   else if(imu_age_clks<CLK_HZ/4)imu_age_clks<=imu_age_clks+1'b1;
 wire [7:0] video_status;
 gao_ov5640_ddr_hdmi #(.IMU_AUTO_START(1)) u_vision(
   .clk50M(clk),.reset_n(reset_n),.imu_recal_n(imu_recal_n),
   .source_key_n(source_key_n),.osd_key_n(1'b1),.hdmi_oe(hdmi_oe),
   .nav_yaw(nav_yaw),.nav_imu_ok(nav_imu_ok),.nav_imu_update(nav_imu_update),
   .motor_left_pwm(),.motor_left_in1(),.motor_left_in2(),
   .motor_right_pwm(),.motor_right_in1(),.motor_right_in2(),
   .motor_stby(),.servo_safe(),
   .rgmii_gtxc(rgmii_gtxc),.rgmii_txd(rgmii_txd),.rgmii_txen(rgmii_txen),
   .eth_mdc(eth_mdc),.eth_rst_n(eth_rst_n),.eth_mdio(eth_mdio),
   .imu_scl(imu_scl),.imu_sda(imu_sda),
   .camera_sclk(camera_sclk),.camera_sdat(camera_sdat),
   .camera_vsync(camera_vsync),.camera_href(camera_href),.camera_pclk(camera_pclk),
   .camera_xclk(camera_xclk),.camera_data(camera_data),
   .camera_rst_n(camera_rst_n),.camera_pwdn(camera_pwdn),.led(video_status),
   .tmds_clk_p(tmds_clk_p),.tmds_clk_n(tmds_clk_n),
   .tmds_data_p(tmds_data_p),.tmds_data_n(tmds_data_n),
   .O_ddr_addr(O_ddr_addr),.O_ddr_ba(O_ddr_ba),.O_ddr_cs_n(O_ddr_cs_n),
   .O_ddr_ras_n(O_ddr_ras_n),.O_ddr_cas_n(O_ddr_cas_n),.O_ddr_we_n(O_ddr_we_n),
   .O_ddr_clk(O_ddr_clk),.O_ddr_clk_n(O_ddr_clk_n),.O_ddr_cke(O_ddr_cke),
   .O_ddr_odt(O_ddr_odt),.O_ddr_reset_n(O_ddr_reset_n),.O_ddr_dqm(O_ddr_dqm),
   .IO_ddr_dq(IO_ddr_dq),.IO_ddr_dqs(IO_ddr_dqs),.IO_ddr_dqs_n(IO_ddr_dqs_n));
 reg [2:0] reset_pipe=0;
 always @(posedge clk or negedge reset_n)
   if(!reset_n)reset_pipe<=0;else reset_pipe<={reset_pipe[1:0],1'b1};
 wire rst_n=reset_pipe[2];
 wire start_press,start_released,stop_press,stop_released;
 button_event #(.STABLE_CYCLES(CLK_HZ/50)) start_key(clk,!rst_n,key_forward_n,start_released,start_press);
 button_event #(.STABLE_CYCLES(CLK_HZ/50)) stop_key(clk,!rst_n,key_reverse_n,stop_released,stop_press);
 reg armed=0;reg [31:0] boot=0;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)begin armed<=0;boot<=0;end
   else if(boot<CLK_HZ/2)boot<=boot+1'b1;
   else if(start_released && stop_released)armed<=1;
 wire stop= !key_reverse_n || !stop_released;
 // The vision subsystem remains the sole I2C owner; export its heading.
 wire signed [31:0] count_l,count_r,speed_l,speed_r,target;
 wire valid_l,valid_r,enable;wire [3:0] phase,code;
 wire signed [31:0] x_q8,y_q8,heading_q16,desired_heading_q16,pass_x_mm;
 wire [15:0] requested_servo_us;
 wire pose_fault,pose_updated,bypass_left;
 wire [7:0] avoid_count;
 wire busy,done,fault,ready;
 wire [15:0] tof_dl,tof_df,tof_dr,tof_min;
 wire [7:0] tof_sl,tof_sf,tof_sr;
 wire [2:0] tof_valid,tof_errors;
 wire [15:0] tof_nl,tof_nf,tof_nr;
 wire tof_configured,tof_all_valid,tof_can_start,tof_halt,tof_slow,tof_latched,tof_brake;
 wire [3:0] tof_reason;
 wire avoid_mode;
 wire accepted_start=start_press && armed && ready && tof_can_start && !stop && !fault;
 tof3_subsystem #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD),.TIMEOUT_MS(TOF_TIMEOUT_MS),
   .INIT_DELAY_MS(TOF_INIT_DELAY_MS),.COMMAND_WAIT_MS(TOF_COMMAND_WAIT_MS)) u_tof(
   .clk(clk),.rst_n(rst_n),.tof_l_rx(tof_l_rx),.tof_f_rx(tof_f_rx),.tof_r_rx(tof_r_rx),
   .tof_l_tx(tof_l_tx),.tof_f_tx(tof_f_tx),.tof_r_tx(tof_r_tx),.configured(tof_configured),
   .distance_l(tof_dl),.distance_f(tof_df),.distance_r(tof_dr),
   .status_l(tof_sl),.status_f(tof_sf),.status_r(tof_sr),.valid(tof_valid),.fault_seen(tof_errors),
   .frames_l(tof_nl),.frames_f(tof_nf),.frames_r(tof_nr));
 // New navigation sensor checks replace the former TOF latch/brake module.
 wire [15:0] tof_min_lf=tof_dl<tof_df ? tof_dl:tof_df;
 assign tof_min=tof_min_lf<tof_dr ? tof_min_lf:tof_dr;
 assign tof_all_valid=tof_configured && (&tof_valid);
 assign tof_can_start=tof_all_valid && imu_fresh;
 assign tof_halt=1'b0;assign tof_brake=1'b0;assign tof_latched=1'b0;
 assign tof_slow=1'b0;assign tof_reason=4'b0;
 wire motor_run=enable && !stop;
 // Preserve old debug names for the existing motor snapshots.
 wire [31:0] progress_l,progress_r,elapsed_ms,fail_progress_l,fail_progress_r;
 wire signed [39:0] balance_error;
 wire [15:0] fresh_l,fresh_r,imbalance_ms;
 wire [3:0] fail_phase;
 assign progress_l=x_q8;assign progress_r=y_q8;assign balance_error=0;
 assign imbalance_ms=0;assign fail_phase=phase;
 assign fail_progress_l=x_q8;assign fail_progress_r=y_q8;
 nav_pose #(.ENC_CPR(ENC_CPR),.WHEEL_UM(WHEEL_UM),.YAW_SIGN(YAW_SIGN)) pose(
   .clk(clk),.rst_n(rst_n),.clear(accepted_start),.track(motor_run),
   .yaw_q16(nav_yaw),.count_l(count_l),.count_r(count_r),.valid_l(valid_l),.valid_r(valid_r),
   .x_q8(x_q8),.y_q8(y_q8),.heading_q16(heading_q16),.updated(pose_updated),.fault(pose_fault));
 nav_controller #(.CLK_HZ(CLK_HZ),.GOAL_MM(GOAL_MM),
   .CENTER_US(SERVO_CENTER_US),.LEFT_US(SERVO_LEFT_US),.RIGHT_US(SERVO_RIGHT_US),
   .CAR_WIDTH_MM(CAR_WIDTH_MM),.CAR_LENGTH_MM(CAR_LENGTH_MM),
   .OBSTACLE_WIDTH_MM(OBSTACLE_WIDTH_MM),.OBSTACLE_LENGTH_MM(OBSTACLE_LENGTH_MM),
   .CLEARANCE_MM(CLEARANCE_MM),.SENSOR_FORWARD_MM(SENSOR_FORWARD_MM),
   .TRIGGER_MM(AVOID_TRIGGER_MM),.AVOID_RPM(AVOID_RPM),.MAX_RPM(MAX_RPM),.WAIT_MS(START_WAIT_MS),
   .LOOK_GAIN_Q8(LOOK_GAIN_Q8),.HEADING_KP_US(HEADING_KP_US)) navigation(
   .clk(clk),.rst_n(rst_n),.start(accepted_start),.stop(stop),
   .x_q8(x_q8),.y_q8(y_q8),.heading_q16(heading_q16),
   .imu_ok(imu_fresh),.imu_update(nav_imu_update),.pose_fault(pose_fault),
   .count_l(count_l),.count_r(count_r),.speed_l(speed_l),.speed_r(speed_r),.valid_l(valid_l),.valid_r(valid_r),
   .tof_l(tof_dl),.tof_f(tof_df),.tof_r(tof_dr),.tof_valid(tof_valid),
   .frames_l(tof_nl),.frames_f(tof_nf),.frames_r(tof_nr),
   .motor_enable(enable),.target_rpm(target),.servo_us(requested_servo_us),
   .phase(phase),.fault_code(code),.busy(busy),.ready(ready),.done(done),.fault(fault),
   .avoid_mode(avoid_mode),.bypass_left(bypass_left),.elapsed_ms(elapsed_ms),
   .desired_heading_q16(desired_heading_q16),.pass_x_mm(pass_x_mm),
   .fresh_l(fresh_l),.fresh_r(fresh_r),.avoid_count(avoid_count));
 wire signed [31:0] motor_target_l,motor_target_r;
 wire signed [15:0] motor_pi_l,motor_pi_r;
 wire [15:0] motor_duty_l,motor_duty_r,invalid_l,invalid_r;
 wire drive_pwma,drive_ain1,drive_ain2,drive_pwmb,drive_bin1,drive_bin2,drive_stby;
 car_closed_loop #(.CLK_FREQ(CLK_HZ),.ENC_PUL_PER_ROT(ENC_CPR),
   .LEFT_TRIM_PERCENT(LEFT_TRIM_PERCENT),.RIGHT_TRIM_PERCENT(RIGHT_TRIM_PERCENT),
   .MOTOR_L_REVERSE(MOTOR_L_REVERSE),.MOTOR_R_REVERSE(MOTOR_R_REVERSE),
   .ENC_L_REVERSE(ENC_L_REVERSE),.ENC_R_REVERSE(ENC_R_REVERSE)) motors(
   .clk(clk),.rst_n(rst_n),.drive_enable(motor_run),.target_rpm(target),.count_left(count_l),.count_right(count_r),
   .speed_left(speed_l),.speed_right(speed_r),.rpm_valid_left(valid_l),.rpm_valid_right(valid_r),
   .ENC_LA(ENC_LA),.ENC_LB(ENC_LB),.ENC_RA(ENC_RA),.ENC_RB(ENC_RB),
   .PWMA(drive_pwma),.AIN1(drive_ain1),.AIN2(drive_ain2),
   .PWMB(drive_pwmb),.BIN1(drive_bin1),.BIN2(drive_bin2),.STBY(drive_stby),.servo_pwm(),
   .debug_target_left(motor_target_l),.debug_target_right(motor_target_r),
   .debug_pi_left(motor_pi_l),.debug_pi_right(motor_pi_r),
   .debug_duty_left(motor_duty_l),.debug_duty_right(motor_duty_r),
   .debug_invalid_left(invalid_l),.debug_invalid_right(invalid_r),.debug_control_valid());
 // Bounded TB6612 short brake after TOF stop, then standby. S0/S3 override it.
 // IN1=IN2=1, PWM=0, STBY=1 is short brake per TB6612 truth table.
 wire output_safe=reset_n && rst_n && !stop && !fault;
 // Register the coherent bridge command before the pad. This breaks the long
 // distance comparator / halt / brake mux path to the physical outputs.
 // Adds one 50MHz cycle (20ns); raw S0/S3 and registered fault still gate pads
 // immediately below, independently of this output register.
 reg [6:0] bridge_command;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)bridge_command<=0;
   else if(!output_safe)bridge_command<=0;
   else if(tof_brake)bridge_command<=7'b1001111;
   else if(motor_run)bridge_command<={drive_stby,drive_pwma,drive_pwmb,
     drive_ain1,drive_ain2,drive_bin1,drive_bin2};
   else bridge_command<=0;
 assign STBY=output_safe && bridge_command[6];
 assign PWMA=output_safe && bridge_command[5];
 assign PWMB=output_safe && bridge_command[4];
 assign AIN1=output_safe && bridge_command[3];
 assign AIN2=output_safe && bridge_command[2];
 assign BIN1=output_safe && bridge_command[1];
 assign BIN2=output_safe && bridge_command[0];
 wire [15:0] pulse=requested_servo_us;
 // No steering pulses at power-up. Latch enable only on an accepted S1 start.
 // On stop/fault remove pulses at the next 20ms frame boundary, never mid-pulse.
 reg servo_session=0;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)servo_session<=0;
   else if(stop || fault)servo_session<=0;
   else if(accepted_start)servo_session<=1;
   else if(tof_halt)servo_session<=0;
 wire servo_internal;
servo_pwm #(.CLK_HZ(CLK_HZ),.CENTER_US(SERVO_CENTER_US),
   .MIN_US(SERVO_LEFT_US),.MAX_US(SERVO_RIGHT_US)) servo(
   .clk(clk),.reset(!rst_n),.enable(servo_session),.request_us(pulse),.pwm(servo_internal));
 assign servo_pwm=reset_n && rst_n && servo_internal;
 reg [31:0] heart;
 always @(posedge clk or negedge rst_n)if(!rst_n)heart<=0;else heart<=heart==CLK_HZ-1 ? 0:heart+1'b1;
 // D0 forward, D1 motor fault, D2 bypass low-speed arc, D3 bypass sequence active, D4 distance done,
 // D5 ready to accept S1, D6 TOF not ready / invalid-data latch, D7 heartbeat.
 reg [7:0] led_status;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)led_status<=0;
    else led_status<={heart<CLK_HZ/2,!tof_can_start,
     (ready && armed && tof_can_start && !stop),done,
     avoid_mode,avoid_mode && bypass_left,fault,(motor_run && target>0)};
 assign led=led_status;

 // Preserve the last motor-control state when enable falls and motor logic resets.
 reg signed [31:0] last_count_l,last_count_r,last_speed_l,last_speed_r,last_target_l,last_target_r;
 reg signed [15:0] last_pi_l,last_pi_r;
 reg [15:0] last_duty_l,last_duty_r,last_invalid_l,last_invalid_r;
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     last_count_l<=0;last_count_r<=0;last_speed_l<=0;last_speed_r<=0;
     last_target_l<=0;last_target_r<=0;last_pi_l<=0;last_pi_r<=0;
     last_duty_l<=0;last_duty_r<=0;last_invalid_l<=0;last_invalid_r<=0;
   end else if(enable)begin
     if(valid_l)begin last_count_l<=count_l;last_speed_l<=speed_l;end
     if(valid_r)begin last_count_r<=count_r;last_speed_r<=speed_r;end
     last_target_l<=motor_target_l;last_target_r<=motor_target_r;
     last_pi_l<=motor_pi_l;last_pi_r<=motor_pi_r;
     last_duty_l<=motor_duty_l;last_duty_r<=motor_duty_r;
     last_invalid_l<=invalid_l;last_invalid_r<=invalid_r;
   end
 end
 // Slow, derived GAO-only clock: 50Hz. The driving controller stays at 50MHz.
 // Snapshot on falling edge; GAO samples 10ms later at rising edge. SDC constrains
 // the generated clock and keeps it related to sys_clk, not an asynchronous cut.
 localparam integer DEBUG_HALF=CLK_HZ/100;
 reg [31:0] debug_div;
 (* keep = 1 *) reg probe_clk;
 (* keep = 1 *) reg [15:0] dbg_version;
 (* keep = 1 *) reg [31:0] dbg_sample,dbg_elapsed;
 (* keep = 1 *) reg [3:0] dbg_phase,dbg_code,dbg_fault_phase;
 (* keep = 1 *) reg [15:0] dbg_flags;
 (* keep = 1 *) reg dbg_terminal;
 (* keep = 1 *) reg signed [31:0] dbg_count_l,dbg_count_r,dbg_rpm_l,dbg_rpm_r,dbg_target_l,dbg_target_r;
 (* keep = 1 *) reg signed [15:0] dbg_pi_l,dbg_pi_r;
 (* keep = 1 *) reg [15:0] dbg_pwm_l,dbg_pwm_r,dbg_invalid_l,dbg_invalid_r,dbg_fresh_l,dbg_fresh_r,dbg_imbalance_ms;
 (* keep = 1 *) reg [31:0] dbg_progress_l,dbg_progress_r,dbg_fault_progress_l,dbg_fault_progress_r;
 (* keep = 1 *) reg signed [39:0] dbg_balance_error;
 (* keep = 1 *) reg [15:0] dbg_tof_l,dbg_tof_f,dbg_tof_r,dbg_tof_min;
 (* keep = 1 *) reg [15:0] dbg_tof_flags;
 (* keep = 1 *) reg [23:0] dbg_tof_status;
 (* keep = 1 *) reg signed [31:0] dbg_x_q8,dbg_y_q8,dbg_heading_q16,dbg_desired_heading_q16,dbg_pass_x_mm;
 (* keep = 1 *) reg [15:0] dbg_servo_us;
 (* keep = 1 *) reg [7:0] dbg_avoid_count;
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     dbg_x_q8<=0;dbg_y_q8<=0;dbg_heading_q16<=0;dbg_desired_heading_q16<=0;dbg_pass_x_mm<=0;
     dbg_servo_us<=SERVO_CENTER_US;dbg_avoid_count<=0;
     debug_div<=0;probe_clk<=0;dbg_version<=16'h2401;dbg_sample<=0;dbg_elapsed<=0;
     dbg_tof_l<=0;dbg_tof_f<=0;dbg_tof_r<=0;dbg_tof_min<=0;dbg_tof_flags<=0;dbg_tof_status<=0;
     dbg_phase<=0;dbg_code<=0;dbg_fault_phase<=0;dbg_flags<=0;dbg_terminal<=0;
     dbg_count_l<=0;dbg_count_r<=0;dbg_rpm_l<=0;dbg_rpm_r<=0;dbg_target_l<=0;dbg_target_r<=0;
     dbg_pi_l<=0;dbg_pi_r<=0;dbg_pwm_l<=0;dbg_pwm_r<=0;dbg_invalid_l<=0;dbg_invalid_r<=0;
     dbg_fresh_l<=0;dbg_fresh_r<=0;dbg_imbalance_ms<=0;dbg_progress_l<=0;dbg_progress_r<=0;
     dbg_fault_progress_l<=0;dbg_fault_progress_r<=0;dbg_balance_error<=0;
   end else if(debug_div==DEBUG_HALF-1)begin
     debug_div<=0;probe_clk<=!probe_clk;
     if(probe_clk)begin
       dbg_x_q8<=x_q8;dbg_y_q8<=y_q8;dbg_heading_q16<=heading_q16;
     dbg_desired_heading_q16<=desired_heading_q16;dbg_pass_x_mm<=pass_x_mm;
     dbg_servo_us<=pulse;dbg_avoid_count<=avoid_count;
     dbg_version<=16'h2401;dbg_sample<=dbg_sample+1'b1;dbg_elapsed<=elapsed_ms;
       dbg_tof_l<=tof_dl;dbg_tof_f<=tof_df;dbg_tof_r<=tof_dr;dbg_tof_min<=tof_min;
       dbg_tof_flags<={tof_reason,tof_errors,tof_brake,tof_latched,tof_slow,tof_can_start,
         tof_configured,tof_valid,accepted_start};
       dbg_tof_status<={tof_sr,tof_sf,tof_sl};
       dbg_phase<=phase;dbg_code<=code;dbg_fault_phase<=fail_phase;
       dbg_flags<={4'b0,ENC_LA,ENC_LB,ENC_RA,ENC_RB,BIN2,BIN1,AIN2,AIN1,armed,ready,busy,enable};
       dbg_terminal<=fault || done || phase==12;
       dbg_count_l<=last_count_l;dbg_count_r<=last_count_r;
       dbg_rpm_l<=last_speed_l;dbg_rpm_r<=last_speed_r;
       dbg_target_l<=last_target_l;dbg_target_r<=last_target_r;
       dbg_pi_l<=last_pi_l;dbg_pi_r<=last_pi_r;dbg_pwm_l<=last_duty_l;dbg_pwm_r<=last_duty_r;
       dbg_invalid_l<=last_invalid_l;dbg_invalid_r<=last_invalid_r;
       dbg_fresh_l<=fresh_l;dbg_fresh_r<=fresh_r;dbg_imbalance_ms<=imbalance_ms;
       dbg_progress_l<=progress_l;dbg_progress_r<=progress_r;dbg_balance_error<=balance_error;
       dbg_fault_progress_l<=fail_progress_l;dbg_fault_progress_r<=fail_progress_r;
     end
   end else debug_div<=debug_div+1'b1;
 end
endmodule
