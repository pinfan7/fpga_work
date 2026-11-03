`timescale 1ns/1ps
// Left-positive degrees Q8 internally; IMU input is degrees Q16.
// A visual observation sets an absolute goal using delayed heading history.
// Between frames IMU feedback, not a fixed startup heading, closes the loop.
module visual_imu_heading #(
 parameter integer CENTER_US=1400,MIN_US=1039,MAX_US=1850,
 parameter integer CAMERA_MIRROR=0,YAW_SIGN=1,
 parameter integer BEARING_Q8_PER_PIXEL=16,HEADING_KP=8,RATE_KD=1,
 parameter integer VISION_LATENCY_MS=100,IMU_STALE_MS=100
)(
 input clk,rst_n,tick_ms,imu_ok,imu_update,
 input signed [31:0] yaw_q16,
 input observation,
 input [11:0] center_x,
 output sensor_ready,output heading_ready,
 output reg [15:0] request_us,
 output reg signed [31:0] heading_q8,goal_q8,error_q8,rate_q8
);
 localparam integer HISTORY_INDEX=VISION_LATENCY_MS/10==0 ? 0:VISION_LATENCY_MS/10-1;
 reg [15:0] age_ms;
 reg [3:0] sample_ms;
 reg [5:0] filled;
 reg signed [31:0] history [0:31];
 reg signed [31:0] previous_heading;
 reg have_previous,have_goal,pending_goal;
 reg signed [31:0] goal_sum,p_term,d_term,control_us;
 reg [2:0] rate_stage;
 reg signed [31:0] rate_difference,rate_delta,raw_rate,clipped_rate,filter_step;
 integer i;
 function signed [31:0] wrap;
   input signed [31:0] a;
   begin
     if(a>32'sd46080)wrap=a-32'sd92160;
     else if(a< -32'sd46080)wrap=a+32'sd92160;
     else wrap=a;
   end
 endfunction
 wire signed [31:0] normalized_yaw=YAW_SIGN==1 ? ($signed(yaw_q16)>>>8): -($signed(yaw_q16)>>>8);
 wire signed [31:0] pixel_error=$signed({1'b0,center_x})-32'sd320;
 // Same 16-pixel visual deadband as V21. Positive image x means right.
 wire signed [31:0] bearing=(pixel_error<=16 && pixel_error>=-16) ? 32'sd0:
   (CAMERA_MIRROR ? pixel_error:-pixel_error)*BEARING_Q8_PER_PIXEL;
 wire history_ready=filled>HISTORY_INDEX;
 assign sensor_ready=imu_ok && age_ms<IMU_STALE_MS;
 assign heading_ready=sensor_ready && history_ready && have_goal;
 always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
     age_ms<=IMU_STALE_MS;sample_ms<=0;filled<=0;
     heading_q8<=0;goal_q8<=0;error_q8<=0;rate_q8<=0;
     previous_heading<=0;have_previous<=0;have_goal<=0;pending_goal<=0;
     goal_sum<=0;p_term<=0;d_term<=0;control_us<=CENTER_US;request_us<=CENTER_US;
     rate_stage<=0;rate_difference<=0;rate_delta<=0;raw_rate<=0;clipped_rate<=0;filter_step<=0;
     for(i=0;i<32;i=i+1)history[i]<=0;
   end else begin
     if(tick_ms && age_ms<IMU_STALE_MS)age_ms<=age_ms+1'b1;
     if(imu_update && imu_ok)begin age_ms<=0;heading_q8<=normalized_yaw;end
     pending_goal<=0;
     if(!sensor_ready)begin
       sample_ms<=0;filled<=0;have_previous<=0;have_goal<=0;rate_q8<=0;
       rate_stage<=0;
       request_us<=CENTER_US;
     end else begin
       if(tick_ms)begin
         if(sample_ms==9)begin
           sample_ms<=0;history[0]<=heading_q8;
           for(i=1;i<32;i=i+1)history[i]<=history[i-1];
           if(filled<32)filled<=filled+1'b1;
           previous_heading<=heading_q8;have_previous<=1;
           if(have_previous)begin
             rate_difference<=heading_q8-previous_heading;rate_stage<=1;
           end
         end else sample_ms<=sample_ms+1'b1;
       end
       // 10ms samples; finish in five fabric clocks (100ns at 50MHz).
       // Separate subtraction/wrap/scaling/clipping/filtering timing paths.
       case(rate_stage)
         1:begin rate_delta<=wrap(rate_difference);rate_stage<=2;end
         2:begin raw_rate<=rate_delta*100;rate_stage<=3;end
         3:begin
           clipped_rate<=raw_rate>92160 ? 32'sd92160:
             raw_rate< -92160 ? -32'sd92160:raw_rate;
           rate_stage<=4;
         end
         4:begin filter_step<=(clipped_rate-rate_q8)>>>2;rate_stage<=5;end
         5:begin rate_q8<=rate_q8+filter_step;rate_stage<=0;end
         default:begin end
       endcase
       if(observation && history_ready)begin
         goal_sum<=(VISION_LATENCY_MS==0 ? heading_q8:history[HISTORY_INDEX])+bearing;
         pending_goal<=1;
       end
       if(pending_goal)begin goal_q8<=wrap(goal_sum);have_goal<=1;end
       // Registered arithmetic stages avoid a combinational sensor-to-pad path.
       error_q8<=have_goal ? wrap(goal_q8-heading_q8):32'sd0;
       p_term<=error_q8*HEADING_KP;
       d_term<=rate_q8*RATE_KD;
       control_us<=CENTER_US-(p_term>>>8)+(d_term>>>8);
       if(heading_ready)begin
         if(control_us<MIN_US)request_us<=MIN_US;
         else if(control_us>MAX_US)request_us<=MAX_US;
         else request_us<=control_us[15:0];
       end else request_us<=CENTER_US;
     end
   end
 end
`ifndef SYNTHESIS
 initial begin
   if(VISION_LATENCY_MS<0 || VISION_LATENCY_MS>320 || VISION_LATENCY_MS%10!=0 ||
      (YAW_SIGN!=1 && YAW_SIGN!=-1) || MIN_US>CENTER_US || MAX_US<CENTER_US)
     $fatal(1,"Invalid visual/IMU heading parameters");
 end
`endif
endmodule
