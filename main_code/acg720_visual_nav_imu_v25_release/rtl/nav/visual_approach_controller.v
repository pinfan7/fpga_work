`timescale 1ns/1ps
// Obstacle-free visual servoing, NOT map-based path planning.
// V25: visual bearing + IMU heading feedback; front TOF is stopping range.
module visual_approach_controller #(
 parameter integer CLK_HZ=50000000,ENC_CPR=1560,WHEEL_UM=65000,
 parameter integer CENTER_US=1400,MIN_US=1039,MAX_US=1850,CAMERA_MIRROR=0,
 parameter integer YAW_SIGN=1,BEARING_Q8_PER_PIXEL=16,HEADING_KP=8,RATE_KD=1,
 parameter integer VISION_LATENCY_MS=100,IMU_STALE_MS=100,
 parameter integer WAIT_MS=1000,TARGET_STALE_MS=250,ENCODER_STALE_MS=300,
 parameter integer NO_FEEDBACK_MS=3000,MAX_TIME_MS=45000,MAX_DISTANCE_MM=3000,
 parameter integer STOP_MM=100,START_MM=150,SIDE_STOP_MM=150,
 parameter integer FAR_RPM=30,NEAR_RPM=15,CREEP_RPM=10,BRAKE_MS=300
)(
 input clk,rst_n,start,stop,source_live,box_update,box_valid,
 input signed [31:0] yaw_q16,input imu_ok,imu_update,
 input [11:0] x0,y0,x1,y1,
 input configured,input [2:0] tof_valid,
 input [15:0] distance_l,distance_f,distance_r,
 input signed [31:0] count_l,count_r,speed_l,speed_r,
 input valid_l,valid_r,
 output motor_enable,output reg signed [31:0] target_rpm,
 output reg [15:0] pulse_us,output [1:0] steering,
 output reg [3:0] phase,fault_code,output busy,done,fault,ready,can_start,brake,
 output reg [31:0] progress_l,progress_r,elapsed_ms,
 output reg [15:0] fresh_l,fresh_r,target_age,
 output reg [11:0] center_x,output reg signed [12:0] error_x,
 output reg [3:0] fault_phase,
 output reg [31:0] fault_progress_l,fault_progress_r,
 output imu_ready,heading_ready,
 output signed [31:0] heading_q8,goal_q8,heading_error_q8,yaw_rate_q8
);
 localparam IDLE=0,PREP=1,TRACK=2,DONE=10,FAULT=11,ABORT=12;
 localparam [63:0] DEN=64'd1*WHEEL_UM*3141593;
 localparam [31:0] MAX_COUNTS=(64'd1000*MAX_DISTANCE_MM*ENC_CPR*1000000+DEN/2)/DEN;
 reg [31:0] divider,zero_l,zero_r,wrong_l,wrong_r,ramp_ms,rpm_mag;
 reg [15:0] brake_ms;
 reg [4:0] slew_ms;
 reg [2:0] stable_frames;
 reg [11:0] previous_center;
 reg have_center,target_present;
 wire tick=divider==CLK_HZ/1000-1;
 wire [12:0] center_sum={1'b0,x0}+{1'b0,x1};
 wire [11:0] incoming_center=center_sum[12:1];
 wire [11:0] jump=incoming_center>previous_center ? incoming_center-previous_center:previous_center-incoming_center;
 wire geometry_ok=box_valid && x0<x1 && y0<y1 && x1<640 && y1<480;
 wire jump_bad=have_center && jump>120;
 wire target_ok=source_live && target_present && target_age<TARGET_STALE_MS;
 wire front_ok=configured && tof_valid[1];
 // Side modules are only a near-contact veto in this obstacle-free first version.
 // Their invalid/out-of-range readings are not interpreted as valid free space.
 wire side_near=(tof_valid[0] && distance_l<=SIDE_STOP_MM) ||
                (tof_valid[2] && distance_r<=SIDE_STOP_MM);
 wire front_near=front_ok && distance_f<=STOP_MM;
 wire hard_halt=!target_ok || !front_ok || side_near || front_near || !heading_ready;
 wire [15:0] heading_pulse;
 visual_imu_heading #(.CENTER_US(CENTER_US),.MIN_US(MIN_US),.MAX_US(MAX_US),
   .CAMERA_MIRROR(CAMERA_MIRROR),.YAW_SIGN(YAW_SIGN),
   .BEARING_Q8_PER_PIXEL(BEARING_Q8_PER_PIXEL),.HEADING_KP(HEADING_KP),.RATE_KD(RATE_KD),
   .VISION_LATENCY_MS(VISION_LATENCY_MS),.IMU_STALE_MS(IMU_STALE_MS)) heading_control(
   .clk(clk),.rst_n(rst_n),.tick_ms(tick),.imu_ok(imu_ok),.imu_update(imu_update),
   .yaw_q16(yaw_q16),.observation(box_update && geometry_ok && source_live && !jump_bad),
   .center_x(incoming_center),.sensor_ready(imu_ready),.heading_ready(heading_ready),
   .request_us(heading_pulse),.heading_q8(heading_q8),.goal_q8(goal_q8),
   .error_q8(heading_error_q8),.rate_q8(yaw_rate_q8));
 assign busy=phase==PREP || phase==TRACK;
 assign done=phase==DONE;
 assign fault=phase==FAULT;
 assign ready=!busy;
 assign can_start=target_ok && stable_frames>=3 && front_ok && distance_f>=START_MM && !side_near && heading_ready;
 assign motor_enable=rst_n && phase==TRACK && !stop && !hard_halt;
 assign brake=rst_n && brake_ms!=0 && !stop;
 assign steering=pulse_us<CENTER_US-12 ? 1:pulse_us>CENTER_US+12 ? 2:0;
 reg signed [31:0] desired_pulse;
 reg [31:0] desired_rpm;
 wire centered=error_x>=-24 && error_x<=24;
 always @*begin
   // IMU closes the heading loop between accepted image observations.
   desired_pulse=heading_pulse;
   desired_rpm=distance_f<=250 ? CREEP_RPM:distance_f<=500 ? NEAR_RPM:FAR_RPM;
   if((!centered || heading_error_q8>2560 || heading_error_q8< -2560) && desired_rpm>NEAR_RPM)desired_rpm=NEAR_RPM;
   target_rpm=motor_enable ? $signed(rpm_mag>desired_rpm ? desired_rpm:rpm_mag):0;
 end
 task fail;
   input [3:0] reason;
   begin
     phase<=FAULT;fault_code<=reason;fault_phase<=phase;
     fault_progress_l<=progress_l;fault_progress_r<=progress_r;
     if(phase==TRACK)brake_ms<=BRAKE_MS;
   end
 endtask
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
   phase<=IDLE;fault_code<=0;divider<=0;elapsed_ms<=0;pulse_us<=CENTER_US;
   progress_l<=0;progress_r<=0;fresh_l<=0;fresh_r<=0;zero_l<=0;zero_r<=0;
   wrong_l<=0;wrong_r<=0;rpm_mag<=CREEP_RPM;ramp_ms<=0;slew_ms<=0;brake_ms<=0;
   target_age<=TARGET_STALE_MS;target_present<=0;stable_frames<=0;have_center<=0;
   previous_center<=320;center_x<=320;error_x<=0;
   fault_phase<=0;fault_progress_l<=0;fault_progress_r<=0;
 end else begin
   divider<=tick?0:divider+1'b1;
   if(tick)begin
     if(target_age<TARGET_STALE_MS)target_age<=target_age+1'b1;
     if(brake_ms!=0)brake_ms<=brake_ms-1'b1;
     if(busy)elapsed_ms<=elapsed_ms+1'b1;
     if(slew_ms==19)begin
       slew_ms<=0;
       if(busy)begin
         if(pulse_us+6<desired_pulse)pulse_us<=pulse_us+6;
         else if(pulse_us>desired_pulse+6)pulse_us<=pulse_us-6;
         else pulse_us<=desired_pulse[15:0];
       end
     end else slew_ms<=slew_ms+1'b1;
   end
   if(!source_live || target_age>=TARGET_STALE_MS)begin
     target_present<=0;stable_frames<=0;have_center<=0;
   end
   if(box_update)begin
     if(geometry_ok && source_live && !jump_bad)begin
       target_present<=1;target_age<=0;previous_center<=incoming_center;have_center<=1;
       center_x<=incoming_center;error_x<=$signed({1'b0,incoming_center})-13'sd320;
       if(stable_frames<3)stable_frames<=stable_frames+1'b1;
     end else begin target_present<=0;stable_frames<=0;have_center<=0;end
   end
   if(phase==TRACK)begin
     if(valid_l)begin
       if(count_l>0)progress_l<=progress_l+count_l;
       fresh_l<=0;zero_l<=count_l==0 ? zero_l+1'b1:0;
       wrong_l<=speed_l< -8 ? wrong_l+1'b1:0;
     end else if(tick && fresh_l<ENCODER_STALE_MS)fresh_l<=fresh_l+1'b1;
     if(valid_r)begin
       if(count_r>0)progress_r<=progress_r+count_r;
       fresh_r<=0;zero_r<=count_r==0 ? zero_r+1'b1:0;
       wrong_r<=speed_r< -8 ? wrong_r+1'b1:0;
     end else if(tick && fresh_r<ENCODER_STALE_MS)fresh_r<=fresh_r+1'b1;
     if(tick)begin
       if(rpm_mag>desired_rpm)rpm_mag<=desired_rpm;
       else if(ramp_ms==99)begin ramp_ms<=0;rpm_mag<=rpm_mag+2>desired_rpm ? desired_rpm:rpm_mag+2;end
       else ramp_ms<=ramp_ms+1'b1;
     end
   end
   case(phase)
     IDLE,DONE,FAULT,ABORT:if(start && can_start && !stop)begin
       phase<=PREP;fault_code<=0;elapsed_ms<=0;brake_ms<=0;
       progress_l<=0;progress_r<=0;fresh_l<=0;fresh_r<=0;zero_l<=0;zero_r<=0;
       wrong_l<=0;wrong_r<=0;rpm_mag<=CREEP_RPM;ramp_ms<=0;
     end
     PREP:if(elapsed_ms>=WAIT_MS)begin phase<=TRACK;elapsed_ms<=0;end
     default:begin end
   endcase
   if(busy)begin
     if(!heading_ready)fail(10);
     else if(!target_ok)fail(1);
     else if(!front_ok)fail(2);
     else if(side_near)fail(7);
     else if(front_near)begin
       if(phase==TRACK && centered)begin phase<=DONE;fault_code<=0;brake_ms<=BRAKE_MS;end
       else fail(8); // Range stop occurred while target was not aligned.
     end else if(phase==TRACK)begin
       if(zero_l>=NO_FEEDBACK_MS/10 || zero_r>=NO_FEEDBACK_MS/10)fail(3);
       else if(wrong_l>=10 || wrong_r>=10)fail(4);
       else if(fresh_l>=ENCODER_STALE_MS || fresh_r>=ENCODER_STALE_MS)fail(5);
       else if(elapsed_ms>=MAX_TIME_MS)fail(6);
       else if(progress_l>=MAX_COUNTS || progress_r>=MAX_COUNTS)fail(9);
     end
   end
   // Manual stop always wins and cancels short-braking as well as motion.
   if(stop)begin
     brake_ms<=0;
     if(busy)begin phase<=ABORT;fault_code<=0;end
   end
 end
endmodule
