`timescale 1ns/1ps
module ackermann_auto_top #(
 parameter integer CLK_HZ=50000000, ENC_CPR=1560,WHEEL_UM=65000,
 parameter integer TURN_PULSE_US=167,LEFT_SMALLER_PULSE=1,
 parameter integer LEFT_TRIM_PERCENT=105,RIGHT_TRIM_PERCENT=100,
 parameter integer MOTOR_L_REVERSE=0,MOTOR_R_REVERSE=0,ENC_L_REVERSE=0,ENC_R_REVERSE=0
)(
 input clk,reset_n,key_forward_n,key_reverse_n,
 input ENC_LA,ENC_LB,ENC_RA,ENC_RB,
 output PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,servo_pwm,
 inout imu_scl,imu_sda,output [7:0] led
);
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
 // IMU remains wired but is completely idle: both bus pins are released.
 // No synthetic IMU status, no calibration, no heading-dependent protection.
 assign imu_scl=1'bz;
 assign imu_sda=1'bz;
 wire signed [31:0] count_l,count_r,speed_l,speed_r,target;
 wire valid_l,valid_r,enable;wire [1:0] steering;wire [3:0] phase,code;
 wire busy,done,fault,ready;
 wire [31:0] progress_l,progress_r,elapsed_ms,fail_progress_l,fail_progress_r;
 wire signed [39:0] balance_error;
 wire [15:0] fresh_l,fresh_r,imbalance_ms;
 wire [3:0] fail_phase;
 auto_sequence #(.CLK_HZ(CLK_HZ),.ENC_CPR(ENC_CPR),.WHEEL_UM(WHEEL_UM),
   .LEFT_TRIM_PERCENT(LEFT_TRIM_PERCENT),.RIGHT_TRIM_PERCENT(RIGHT_TRIM_PERCENT)) sequence_control(
   .clk(clk),.rst_n(rst_n),.start(start_press && armed),.stop(stop),
   .count_l(count_l),.count_r(count_r),
   .speed_l(speed_l),.speed_r(speed_r),.valid_l(valid_l),.valid_r(valid_r),
   .motor_enable(enable),.target_rpm(target),.steering(steering),
   .phase(phase),.fault_code(code),.busy(busy),.done(done),.fault(fault),.ready(ready),
   .debug_progress_left(progress_l),.debug_progress_right(progress_r),.debug_balance_error(balance_error),
   .debug_elapsed(elapsed_ms),.debug_fresh_left(fresh_l),.debug_fresh_right(fresh_r),
   .debug_imbalance_ms(imbalance_ms),.fault_phase(fail_phase),
   .fault_progress_left(fail_progress_l),.fault_progress_right(fail_progress_r));
 wire signed [31:0] motor_target_l,motor_target_r;
 wire signed [15:0] motor_pi_l,motor_pi_r;
 wire [15:0] motor_duty_l,motor_duty_r,invalid_l,invalid_r;
 car_closed_loop #(.CLK_FREQ(CLK_HZ),.ENC_PUL_PER_ROT(ENC_CPR),
   .LEFT_TRIM_PERCENT(LEFT_TRIM_PERCENT),.RIGHT_TRIM_PERCENT(RIGHT_TRIM_PERCENT),
   .MOTOR_L_REVERSE(MOTOR_L_REVERSE),.MOTOR_R_REVERSE(MOTOR_R_REVERSE),
   .ENC_L_REVERSE(ENC_L_REVERSE),.ENC_R_REVERSE(ENC_R_REVERSE)) motors(
   .clk(clk),.rst_n(rst_n),.drive_enable(enable),.target_rpm(target),.count_left(count_l),.count_right(count_r),
   .speed_left(speed_l),.speed_right(speed_r),.rpm_valid_left(valid_l),.rpm_valid_right(valid_r),
   .ENC_LA(ENC_LA),.ENC_LB(ENC_LB),.ENC_RA(ENC_RA),.ENC_RB(ENC_RB),
   .PWMA(PWMA),.AIN1(AIN1),.AIN2(AIN2),.PWMB(PWMB),.BIN1(BIN1),.BIN2(BIN2),.STBY(STBY),.servo_pwm(),
   .debug_target_left(motor_target_l),.debug_target_right(motor_target_r),
   .debug_pi_left(motor_pi_l),.debug_pi_right(motor_pi_r),
   .debug_duty_left(motor_duty_l),.debug_duty_right(motor_duty_r),
   .debug_invalid_left(invalid_l),.debug_invalid_right(invalid_r),.debug_control_valid());
 wire smaller=(steering==1)==(LEFT_SMALLER_PULSE!=0);
 wire [15:0] pulse=steering==0 ? 1500:smaller ? 1500-TURN_PULSE_US:1500+TURN_PULSE_US;
 // No steering pulses at power-up. Latch enable only on an accepted S1 start.
 // On stop/fault remove pulses at the next 20ms frame boundary, never mid-pulse.
 reg servo_session=0;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)servo_session<=0;
   else if(stop || fault)servo_session<=0;
   else if(start_press && armed && ready)servo_session<=1;
 wire servo_internal;
 servo_pwm #(.CLK_HZ(CLK_HZ),.MIN_US(1333),.MAX_US(1667)) servo(
   .clk(clk),.reset(!rst_n),.enable(servo_session),.request_us(pulse),.pwm(servo_internal));
 assign servo_pwm=reset_n && rst_n && servo_internal;
 reg [31:0] heart;
 always @(posedge clk or negedge rst_n)if(!rst_n)heart<=0;else heart<=heart==CLK_HZ-1 ? 0:heart+1'b1;
 // Fault mode: D3..D0 display a binary code. D6 is latched fault.
 reg [7:0] led_status;
 always @(posedge clk or negedge rst_n)
   if(!rst_n)led_status<=0;
   else led_status<={heart<CLK_HZ/2,fault,(ready && armed) || (phase==1 && heart<CLK_HZ/4),done,
     (fault ? code:{steering==2,steering==1,enable && target<0,enable && target>0})};
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
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     debug_div<=0;probe_clk<=0;dbg_version<=16'h0602;dbg_sample<=0;dbg_elapsed<=0;
     dbg_phase<=0;dbg_code<=0;dbg_fault_phase<=0;dbg_flags<=0;dbg_terminal<=0;
     dbg_count_l<=0;dbg_count_r<=0;dbg_rpm_l<=0;dbg_rpm_r<=0;dbg_target_l<=0;dbg_target_r<=0;
     dbg_pi_l<=0;dbg_pi_r<=0;dbg_pwm_l<=0;dbg_pwm_r<=0;dbg_invalid_l<=0;dbg_invalid_r<=0;
     dbg_fresh_l<=0;dbg_fresh_r<=0;dbg_imbalance_ms<=0;dbg_progress_l<=0;dbg_progress_r<=0;
     dbg_fault_progress_l<=0;dbg_fault_progress_r<=0;dbg_balance_error<=0;
   end else if(debug_div==DEBUG_HALF-1)begin
     debug_div<=0;probe_clk<=!probe_clk;
     if(probe_clk)begin
       dbg_version<=16'h0602;dbg_sample<=dbg_sample+1'b1;dbg_elapsed<=elapsed_ms;
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
