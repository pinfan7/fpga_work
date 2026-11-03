`timescale 1ns/1ps
module ackermann_auto_top #(
 parameter integer CLK_HZ=50000000, ENC_CPR=1560,WHEEL_UM=65000,
 parameter integer TURN_PULSE_US=167,LEFT_SMALLER_PULSE=1,
 parameter integer LEFT_TRIM_PERCENT=105,RIGHT_TRIM_PERCENT=100,
 parameter integer TOF_BAUD=9600,TOF_TIMEOUT_MS=500,
 parameter integer TOF_SLOW_MM=700,TOF_STOP_MM=350,
 parameter integer TOF_SLOW_CLEAR_MM=750,TOF_RESTART_MM=400,
 parameter integer TOF_CLEAR_MS=300,TOF_BRAKE_MS=300,TOF_SLOW_RPM=20,
 parameter integer TOF_INIT_DELAY_MS=1000,TOF_COMMAND_WAIT_MS=150,
 parameter integer NAV_STOP_MM=100,NAV_WAIT_MS=1000,SERVO_CENTER_US=1400,CAMERA_MIRROR=0,
 parameter integer SERVO_LEFT_US=1039,SERVO_RIGHT_US=1850,YAW_SIGN=1,
 parameter integer VISION_LATENCY_MS=100,BEARING_Q8_PER_PIXEL=16,HEADING_KP=8,RATE_KD=1,
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
 // Video processing stays independent. Only a coherent target mailbox and a
 // complete-frame freshness signal enter the navigation controller.
 wire [7:0] video_status;
 wire nav_update,nav_valid,nav_live,nav_imu_ok,nav_imu_update;
 wire signed [31:0] nav_yaw;
 wire [11:0] nav_x0,nav_y0,nav_x1,nav_y1;
 gao_ov5640_ddr_hdmi #(.IMU_AUTO_START(1)) u_vision(
   .nav_update(nav_update),.nav_valid(nav_valid),.nav_live(nav_live),
   .nav_yaw(nav_yaw),.nav_imu_ok(nav_imu_ok),.nav_imu_update(nav_imu_update),
   .nav_x0(nav_x0),.nav_y0(nav_y0),.nav_x1(nav_x1),.nav_y1(nav_y1),
   .clk50M(clk),.reset_n(reset_n),.imu_recal_n(imu_recal_n),
   .source_key_n(source_key_n),.osd_key_n(1'b1),.hdmi_oe(hdmi_oe),
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
 // Video/IMU subsystem exclusively owns I2C. Export its calibrated yaw
 // in this same board-clock domain for the visual heading feedback loop.
 wire signed [31:0] count_l,count_r,speed_l,speed_r,target;
 wire valid_l,valid_r,enable;wire [1:0] steering;wire [3:0] phase,code;
 wire busy,done,fault,ready;
 wire [15:0] tof_dl,tof_df,tof_dr,tof_min;
 wire [7:0] tof_sl,tof_sf,tof_sr;
 wire [2:0] tof_valid,tof_errors;
 wire [15:0] tof_nl,tof_nf,tof_nr;
 wire tof_configured,tof_all_valid,tof_can_start,tof_halt,tof_slow,tof_latched,tof_brake;
 wire [3:0] tof_reason;
 wire accepted_start=start_press && armed && ready && tof_can_start && !stop;
 tof3_subsystem #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD),.TIMEOUT_MS(TOF_TIMEOUT_MS),
   .INIT_DELAY_MS(TOF_INIT_DELAY_MS),.COMMAND_WAIT_MS(TOF_COMMAND_WAIT_MS)) u_tof(
   .clk(clk),.rst_n(rst_n),.tof_l_rx(tof_l_rx),.tof_f_rx(tof_f_rx),.tof_r_rx(tof_r_rx),
   .tof_l_tx(tof_l_tx),.tof_f_tx(tof_f_tx),.tof_r_tx(tof_r_tx),.configured(tof_configured),
   .distance_l(tof_dl),.distance_f(tof_df),.distance_r(tof_dr),
   .status_l(tof_sl),.status_f(tof_sf),.status_r(tof_sr),.valid(tof_valid),.fault_seen(tof_errors),
   .frames_l(tof_nl),.frames_f(tof_nf),.frames_r(tof_nr));
 assign tof_min=tof_dl<tof_df ? (tof_dl<tof_dr ? tof_dl:tof_dr):(tof_df<tof_dr ? tof_df:tof_dr);
 assign tof_all_valid=tof_configured && (&tof_valid);
 assign tof_slow=tof_valid[1] && tof_df<=500;
 assign tof_latched=fault;
 assign tof_reason=code;
 assign tof_halt=fault || done;
 wire motor_run=enable && !tof_halt && !stop;
 wire [31:0] progress_l,progress_r,elapsed_ms,fail_progress_l,fail_progress_r;
 wire signed [39:0] balance_error;
 wire [15:0] fresh_l,fresh_r,imbalance_ms;
 wire [3:0] fail_phase;
 wire [15:0] nav_pulse,target_age;
 wire [11:0] nav_center;
 wire signed [12:0] nav_error;
 wire nav_imu_ready,nav_heading_ready;
 wire signed [31:0] nav_heading_q8,nav_goal_q8,nav_heading_error_q8,nav_rate_q8;
 assign balance_error=0;
 assign imbalance_ms=0;
 visual_approach_controller #(.CLK_HZ(CLK_HZ),.ENC_CPR(ENC_CPR),.WHEEL_UM(WHEEL_UM),
   .CENTER_US(SERVO_CENTER_US),.MIN_US(SERVO_LEFT_US),.MAX_US(SERVO_RIGHT_US),
   .CAMERA_MIRROR(CAMERA_MIRROR),.YAW_SIGN(YAW_SIGN),.VISION_LATENCY_MS(VISION_LATENCY_MS),
   .BEARING_Q8_PER_PIXEL(BEARING_Q8_PER_PIXEL),.HEADING_KP(HEADING_KP),.RATE_KD(RATE_KD),
   .WAIT_MS(NAV_WAIT_MS),.STOP_MM(NAV_STOP_MM)) sequence_control(
   .clk(clk),.rst_n(rst_n),.start(accepted_start),.stop(stop),
   .source_live(nav_live),.box_update(nav_update),.box_valid(nav_valid),
   .yaw_q16(nav_yaw),.imu_ok(nav_imu_ok),.imu_update(nav_imu_update),
   .x0(nav_x0),.y0(nav_y0),.x1(nav_x1),.y1(nav_y1),
   .configured(tof_configured),.tof_valid(tof_valid),
   .distance_l(tof_dl),.distance_f(tof_df),.distance_r(tof_dr),
   .count_l(count_l),.count_r(count_r),
   .speed_l(speed_l),.speed_r(speed_r),.valid_l(valid_l),.valid_r(valid_r),
   .motor_enable(enable),.target_rpm(target),.steering(steering),.pulse_us(nav_pulse),
   .phase(phase),.fault_code(code),.busy(busy),.done(done),.fault(fault),.ready(ready),
   .can_start(tof_can_start),.brake(tof_brake),
   .progress_l(progress_l),.progress_r(progress_r),
   .elapsed_ms(elapsed_ms),.fresh_l(fresh_l),.fresh_r(fresh_r),
   .target_age(target_age),.center_x(nav_center),.error_x(nav_error),.fault_phase(fail_phase),
   .fault_progress_l(fail_progress_l),.fault_progress_r(fail_progress_r),
   .imu_ready(nav_imu_ready),.heading_ready(nav_heading_ready),.heading_q8(nav_heading_q8),
   .goal_q8(nav_goal_q8),.heading_error_q8(nav_heading_error_q8),.yaw_rate_q8(nav_rate_q8));
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
 wire output_safe=reset_n && rst_n && !stop;
 // Register the coherent bridge command before the pad. This breaks the long
 // distance comparator / halt / brake mux path to the physical outputs.
 // Adds one 50MHz cycle (20ns). Raw S0/S3 gate pads immediately below;
 // navigation faults cancel run and request the bounded short-brake sequence.
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
 wire [15:0] pulse=LEFT_SMALLER_PULSE ? nav_pulse:2*SERVO_CENTER_US-nav_pulse;
 // No steering pulses at power-up. Latch enable only on an accepted S1 start.
 // On stop/fault remove pulses at the next 20ms frame boundary, never mid-pulse.
 reg servo_session=0;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)servo_session<=0;
   else if(stop)servo_session<=0;
   else if(accepted_start)servo_session<=1;
   else if(tof_halt)servo_session<=0;
 wire servo_internal;
 servo_pwm #(.CLK_HZ(CLK_HZ),.CENTER_US(SERVO_CENTER_US),.MIN_US(SERVO_LEFT_US),.MAX_US(SERVO_RIGHT_US)) servo(
   .clk(clk),.reset(!rst_n),.enable(servo_session),.request_us(pulse),.pwm(servo_internal));
 assign servo_pwm=reset_n && rst_n && servo_internal;
 reg [31:0] heart;
 always @(posedge clk or negedge rst_n)if(!rst_n)heart<=0;else heart<=heart==CLK_HZ-1 ? 0:heart+1'b1;
 // D0 forward, D1 fault, D2 close-range slow, D3 preparing steering,
 // D4 arrived, D5 ready for S1, D6 vision/front-range/IMU not ready, D7 heartbeat.
 reg [7:0] led_status;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)led_status<=0;
   else led_status<={heart<CLK_HZ/2,(!nav_live || !nav_valid || !tof_valid[1] || !nav_heading_ready),
     (ready && armed && tof_can_start && !stop),done,
     (phase==1),(tof_slow && tof_valid[1]),fault,(motor_run && target>0)};
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
 (* keep = 1 *) reg [15:0] dbg_nav_pulse,dbg_nav_age;
 (* keep = 1 *) reg [11:0] dbg_nav_center;
 (* keep = 1 *) reg signed [12:0] dbg_nav_error;
 (* keep = 1 *) reg signed [31:0] dbg_heading_q8,dbg_goal_q8,dbg_heading_error_q8,dbg_rate_q8;
 (* keep = 1 *) reg [3:0] dbg_imu_flags;
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     debug_div<=0;probe_clk<=0;dbg_version<=16'h2501;dbg_sample<=0;dbg_elapsed<=0;
     dbg_heading_q8<=0;dbg_goal_q8<=0;dbg_heading_error_q8<=0;dbg_rate_q8<=0;dbg_imu_flags<=0;
     dbg_nav_pulse<=SERVO_CENTER_US;dbg_nav_age<=0;dbg_nav_center<=320;dbg_nav_error<=0;
     dbg_tof_l<=0;dbg_tof_f<=0;dbg_tof_r<=0;dbg_tof_min<=0;dbg_tof_flags<=0;dbg_tof_status<=0;
     dbg_phase<=0;dbg_code<=0;dbg_fault_phase<=0;dbg_flags<=0;dbg_terminal<=0;
     dbg_count_l<=0;dbg_count_r<=0;dbg_rpm_l<=0;dbg_rpm_r<=0;dbg_target_l<=0;dbg_target_r<=0;
     dbg_pi_l<=0;dbg_pi_r<=0;dbg_pwm_l<=0;dbg_pwm_r<=0;dbg_invalid_l<=0;dbg_invalid_r<=0;
     dbg_fresh_l<=0;dbg_fresh_r<=0;dbg_imbalance_ms<=0;dbg_progress_l<=0;dbg_progress_r<=0;
     dbg_fault_progress_l<=0;dbg_fault_progress_r<=0;dbg_balance_error<=0;
   end else if(debug_div==DEBUG_HALF-1)begin
     debug_div<=0;probe_clk<=!probe_clk;
     if(probe_clk)begin
       dbg_version<=16'h2501;dbg_sample<=dbg_sample+1'b1;dbg_elapsed<=elapsed_ms;
       dbg_heading_q8<=nav_heading_q8;dbg_goal_q8<=nav_goal_q8;
       dbg_heading_error_q8<=nav_heading_error_q8;dbg_rate_q8<=nav_rate_q8;
       dbg_imu_flags<={nav_heading_ready,nav_imu_ready,nav_imu_ok,nav_imu_update};
       dbg_nav_pulse<=nav_pulse;dbg_nav_age<=target_age;dbg_nav_center<=nav_center;dbg_nav_error<=nav_error;
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
