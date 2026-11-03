`timescale 1ns/1ps
// Short indoor trial: one compact stationary obstacle at a time, three +/-45deg
// TOFs. IMU yaw is relative to S1 start. Stop rather than claim a false return.
module nav_controller #(
 parameter integer CLK_HZ=50000000,GOAL_MM=2000,
 parameter integer CENTER_US=1400,LEFT_US=1039,RIGHT_US=1850,
 parameter integer CAR_WIDTH_MM=200,CAR_LENGTH_MM=270,
 parameter integer OBSTACLE_WIDTH_MM=88,OBSTACLE_LENGTH_MM=88,
 parameter integer CLEARANCE_MM=60,SENSOR_FORWARD_MM=270,
 parameter integer TRIGGER_MM=400,SIDE_CLEAR_MM=500,EMERGENCY_MM=100,
 parameter integer MAX_RPM=80,AVOID_RPM=20,SLOW_RPM=20,
 parameter integer WAIT_MS=1000,CONFIRM_SCANS=3,
 parameter integer Y_TOL_MM=25,HEADING_TOL_DEG=3,
 parameter integer LOOK_GAIN_Q8=128,MAX_HEADING_DEG=40,HEADING_KP_US=64,
 parameter integer STAGE_TIMEOUT_MS=30000,IMU_STALE_MS=250,ENCODER_STALE_MS=300
)(
 input clk,rst_n,start,stop,
 input signed [31:0] x_q8,y_q8,heading_q16,
 input imu_ok,imu_update,pose_fault,
 input signed [31:0] count_l,count_r,speed_l,speed_r,
 input valid_l,valid_r,
 input [15:0] tof_l,tof_f,tof_r,frames_l,frames_f,frames_r,
 input [2:0] tof_valid,
 output wire motor_enable,busy,ready,done,fault,avoid_mode,
 output reg signed [31:0] target_rpm,
 output reg [15:0] servo_us,
 output reg [3:0] phase,fault_code,
 output reg bypass_left,
 output reg [31:0] elapsed_ms,
 output reg signed [31:0] desired_heading_q16,pass_x_mm,
 output reg [15:0] fresh_l,fresh_r,
 output reg [7:0] avoid_count
);
 localparam IDLE=0,PREP=1,CRUISE=2,SHIFT_OUT=3,PASS=4,RETURN=5,
            DONE=10,FAULT=11,ABORT=12;
 localparam integer LANE_MM=CAR_WIDTH_MM/2+OBSTACLE_WIDTH_MM+CLEARANCE_MM;
 reg [31:0] divider,stage_ms;
 reg [15:0] imu_age,zero_l_ms,zero_r_ms;
 reg [15:0] last_l,last_f,last_r;
 reg [7:0] confirm_count,clear_count;
 reg [7:0] ramp_ms;
 reg [31:0] rpm_mag;
 wire tick=divider==CLK_HZ/1000-1;
 wire moving=phase==CRUISE || phase==SHIFT_OUT || phase==PASS || phase==RETURN;
 assign motor_enable=rst_n && moving && !stop && !fault && imu_ok && (&tof_valid);
 assign busy=phase==PREP || moving;
 assign ready=phase==IDLE || phase==DONE || phase==ABORT;
 assign done=phase==DONE;
 assign fault=phase==FAULT;
 assign avoid_mode=phase==SHIFT_OUT || phase==PASS || phase==RETURN;
 wire scan=(frames_l!=last_l && frames_f!=last_f && frames_r!=last_r);
 wire threat=(tof_l<=TRIGGER_MM || tof_f<=TRIGGER_MM || tof_r<=TRIGGER_MM);
 wire left_clear=tof_l>=SIDE_CLEAR_MM;
 wire right_clear=tof_r>=SIDE_CLEAR_MM;
 wire hard_close=tof_f<=EMERGENCY_MM || tof_l<=EMERGENCY_MM || tof_r<=EMERGENCY_MM;
 wire signed [31:0] x_mm=$signed(x_q8)>>>8;
 wire signed [31:0] y_mm=$signed(y_q8)>>>8;
 wire aligned=y_q8<=Y_TOL_MM*256 && y_q8>=-Y_TOL_MM*256 &&
                  heading_q16<=HEADING_TOL_DEG*65536 && heading_q16>=-HEADING_TOL_DEG*65536;
 wire lane_aligned=(bypass_left ?
                   (y_mm>=LANE_MM-Y_TOL_MM && y_mm<=LANE_MM+Y_TOL_MM):
                   (y_mm>=-LANE_MM-Y_TOL_MM && y_mm<=-LANE_MM+Y_TOL_MM)) &&
                   heading_q16<=HEADING_TOL_DEG*65536 && heading_q16>=-HEADING_TOL_DEG*65536;
 reg signed [31:0] lane_q8,error_q16;
 reg signed [47:0] lateral_command;
 reg signed [63:0] pulse_correction;
 reg signed [31:0] pulse_command;
 reg [31:0] desired_rpm;
 reg [15:0] hit_forward_mm;
 always @*begin
   // A +/-45-degree beam's forward projection is distance/sqrt(2).
   hit_forward_mm=TRIGGER_MM;
   if(tof_f<=TRIGGER_MM)hit_forward_mm=tof_f;
   if(tof_l<=TRIGGER_MM && (tof_l*32'd724)/1024<hit_forward_mm)
      hit_forward_mm=(tof_l*32'd724)/1024;
   if(tof_r<=TRIGGER_MM && (tof_r*32'd724)/1024<hit_forward_mm)
      hit_forward_mm=(tof_r*32'd724)/1024;
   lane_q8=0;
   if(phase==SHIFT_OUT || phase==PASS)lane_q8=(bypass_left ? LANE_MM:-LANE_MM)*256;
   lateral_command=$signed(lane_q8-y_q8)*LOOK_GAIN_Q8;
   if(lateral_command>MAX_HEADING_DEG*65536)desired_heading_q16=MAX_HEADING_DEG*65536;
   else if(lateral_command< -MAX_HEADING_DEG*65536)desired_heading_q16=-MAX_HEADING_DEG*65536;
   else desired_heading_q16=lateral_command[31:0];
   error_q16=desired_heading_q16-heading_q16;
   pulse_correction=$signed(error_q16)*HEADING_KP_US;
   pulse_command=CENTER_US-($signed(pulse_correction)>>>16);
   if(!moving)servo_us=CENTER_US;
   else if(pulse_command<LEFT_US)servo_us=LEFT_US;
   else if(pulse_command>RIGHT_US)servo_us=RIGHT_US;
   else servo_us=pulse_command[15:0];
   desired_rpm=avoid_mode ? AVOID_RPM:MAX_RPM;
   if(phase==CRUISE && (tof_l<800 || tof_f<800 || tof_r<800 || x_mm>=GOAL_MM-200))desired_rpm=SLOW_RPM;
   target_rpm=!motor_enable ? 0:$signed(rpm_mag>desired_rpm ? desired_rpm:rpm_mag);
 end
 task enter;
   input [3:0] p;
   begin phase<=p;stage_ms<=0;confirm_count<=0;clear_count<=0;end
 endtask
 task fail;
   input [3:0] c;
   begin phase<=FAULT;fault_code<=c;end
 endtask
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     phase<=IDLE;fault_code<=0;bypass_left<=1;elapsed_ms<=0;stage_ms<=0;divider<=0;
     imu_age<=IMU_STALE_MS;fresh_l<=0;fresh_r<=0;zero_l_ms<=0;zero_r_ms<=0;
     last_l<=0;last_f<=0;last_r<=0;confirm_count<=0;clear_count<=0;
     rpm_mag<=10;ramp_ms<=0;pass_x_mm<=0;avoid_count<=0;
   end else begin
     divider<=tick ? 0:divider+1'b1;
     if(imu_update && imu_ok)imu_age<=0;
     else if(tick && imu_age<IMU_STALE_MS)imu_age<=imu_age+1'b1;
     if(start && ready)begin
       enter(PREP);fault_code<=0;elapsed_ms<=0;avoid_count<=0;
       rpm_mag<=10;ramp_ms<=0;fresh_l<=0;fresh_r<=0;zero_l_ms<=0;zero_r_ms<=0;
       last_l<=frames_l;last_f<=frames_f;last_r<=frames_r;
     end
     if(tick && busy)begin elapsed_ms<=elapsed_ms+1'b1;stage_ms<=stage_ms+1'b1;end
     if(moving)begin
       if(valid_l)begin
         fresh_l<=0;
         zero_l_ms<=count_l==0 ? zero_l_ms+10:0;
       end else if(tick && fresh_l<ENCODER_STALE_MS)fresh_l<=fresh_l+1'b1;
       if(valid_r)begin
         fresh_r<=0;
         zero_r_ms<=count_r==0 ? zero_r_ms+10:0;
       end else if(tick && fresh_r<ENCODER_STALE_MS)fresh_r<=fresh_r+1'b1;
       if(tick)begin
         if(rpm_mag>desired_rpm)rpm_mag<=desired_rpm;
         else if(ramp_ms==99)begin
           ramp_ms<=0;rpm_mag<=rpm_mag+5>desired_rpm ? desired_rpm:rpm_mag+5;
         end else ramp_ms<=ramp_ms+1'b1;
       end
     end
     if(tick)begin
       if(scan)begin last_l<=frames_l;last_f<=frames_f;last_r<=frames_r;end
       case(phase)
         PREP:if(stage_ms>=WAIT_MS)begin
           enter(CRUISE);fresh_l<=0;fresh_r<=0;zero_l_ms<=0;zero_r_ms<=0;
         end
         CRUISE:begin
           if(scan)begin
             if(!threat)confirm_count<=0;
             else if(confirm_count<CONFIRM_SCANS)confirm_count<=confirm_count+1'b1;
             if(threat && confirm_count>=CONFIRM_SCANS-1)begin
               if(!left_clear && !right_clear)fail(2);
               else begin
                 // Choose once per obstacle. Invalid ranges are never free space.
                 bypass_left<=left_clear && (!right_clear || tof_l+32'd100>=tof_r);
                 pass_x_mm<=x_mm+SENSOR_FORWARD_MM+hit_forward_mm+
                              OBSTACLE_LENGTH_MM+CAR_LENGTH_MM+CLEARANCE_MM;
                 avoid_count<=avoid_count+1'b1;enter(SHIFT_OUT);
               end
             end
           end
         end
         SHIFT_OUT:if(lane_aligned)enter(PASS);
         PASS:begin
           // Only return after the conservative obstacle tail bound AND fresh
           // forward clearance. Absence from a diagonal beam alone is insufficient.
           if(scan)begin
             if(tof_f<SIDE_CLEAR_MM)clear_count<=0;
             else if(clear_count<CONFIRM_SCANS)clear_count<=clear_count+1'b1;
           end
           if(x_mm>=pass_x_mm && clear_count>=CONFIRM_SCANS)enter(RETURN);
         end
         RETURN:if(aligned)enter(CRUISE);
         default:begin end
       endcase
       if(moving && x_mm>=GOAL_MM)begin
         if(aligned && !avoid_mode && !threat)enter(DONE);
         else fail(13);
       end
       if(busy)begin
         if(!(&tof_valid))fail(1);
         if(!imu_ok || imu_age>=IMU_STALE_MS)fail(8);
         if(stage_ms>=STAGE_TIMEOUT_MS)fail(6);
         if(pose_fault)fail(7);
         if(moving)begin
           if(hard_close)fail(2);
           if(fresh_l>=ENCODER_STALE_MS || fresh_r>=ENCODER_STALE_MS)fail(5);
           if(zero_l_ms>=3000 || zero_r_ms>=3000)fail(3);
           if(heading_q16>75*65536 || heading_q16< -75*65536)fail(9);
           if(y_mm>LANE_MM+200 || y_mm< -LANE_MM-200)fail(14);
         end
       end
     end
     // Fault wins over a state transition occurring on the same encoder sample.
     if(moving && ((valid_l && (count_l< -2 || speed_l< -8)) ||
                   (valid_r && (count_r< -2 || speed_r< -8))))fail(4);
     if(stop && busy)enter(ABORT);
   end
 end
endmodule
