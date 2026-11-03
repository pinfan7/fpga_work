`timescale 1ns/1ps
// Encoder-distance test. IMU is intentionally absent from this controller.
// 1m forward, 1m reverse, 1m left arc, retrace that arc in reverse, then 1m right arc.
// Encoder distance approximates retracing; it is not pose-controlled.
module auto_sequence #(
 parameter integer CLK_HZ=50000000, ENC_CPR=1560, WHEEL_UM=65000,
 parameter integer DIST_MM=1000, MAX_RPM=60, SLOW_RPM=20,
 parameter integer WAIT_MS=1000, ENCODER_STALE_MS=300,
 parameter integer NO_FEEDBACK_MS=3000, SEGMENT_TIMEOUT_MS=30000,
 parameter integer LEFT_TRIM_PERCENT=105, RIGHT_TRIM_PERCENT=100,
 parameter integer IMBALANCE_GRACE_MS=500, IMBALANCE_HOLD_MS=150
)(
 input clk,rst_n,start,stop,
 input signed [31:0] count_l,count_r,speed_l,speed_r,
 input valid_l,valid_r,
 output motor_enable,output reg signed [31:0] target_rpm,
 output reg [1:0] steering,
 output reg [3:0] phase,fault_code,output busy,done,fault,ready,
 output wire [31:0] debug_progress_left,debug_progress_right,
 output wire signed [39:0] debug_balance_error,
 output wire [31:0] debug_elapsed,
 output wire [15:0] debug_fresh_left,debug_fresh_right,debug_imbalance_ms,
 output reg [3:0] fault_phase,
 output reg [31:0] fault_progress_left,fault_progress_right
);
 localparam IDLE=0,PREP=1,FWD=2,WAIT_BACK=3,BACK=4,LEFT=5,
            LEFT_RETRACE=6,RIGHT=7,RIGHT_RETRACE=8,
            FINISH=9,DONE=10,FAULT=11,ABORT=12;
 localparam [63:0] DEN=64'd1*WHEEL_UM*3141593;
 localparam [31:0] GOAL=(64'd1000*DIST_MM*ENC_CPR*1000000+DEN/2)/DEN;
 localparam [31:0] SLOW_COUNTS=(64'd1000*150*ENC_CPR*1000000+DEN/2)/DEN;
 reg [31:0] divider,elapsed,ramp_ms,rpm_mag;
 reg [31:0] progress_l,progress_r,zero_l,zero_r,wrong_l,wrong_r;
 reg [31:0] fresh_l,fresh_r;
 reg signed [39:0] balance_l,balance_r;
 reg [15:0] imbalance_ms;
 // Units: counts * RIGHT_TRIM_PERCENT. Remove the intended speed ratio.
 localparam [39:0] BALANCE_LIMIT=40'd1*GOAL*RIGHT_TRIM_PERCENT/4;
 wire signed [39:0] balance_error=balance_l-balance_r;
 wire balance_bad=balance_error>$signed(BALANCE_LIMIT) || balance_error< -$signed(BALANCE_LIMIT);
 assign debug_progress_left=progress_l;
 assign debug_progress_right=progress_r;
 assign debug_balance_error=balance_error;
 assign debug_elapsed=elapsed;
 assign debug_fresh_left=fresh_l[15:0];
 assign debug_fresh_right=fresh_r[15:0];
 assign debug_imbalance_ms=imbalance_ms;
 wire tick=(divider==CLK_HZ/1000-1);
 wire arc_phase=phase==LEFT || phase==LEFT_RETRACE || phase==RIGHT || phase==RIGHT_RETRACE;
 wire moving=phase==FWD || phase==BACK ||
            (arc_phase && (phase==RIGHT ? elapsed>=2*WAIT_MS :
                                          elapsed>=WAIT_MS));
 assign busy=phase>=PREP && phase<=FINISH;
 assign done=phase==DONE;
 assign fault=phase==FAULT;
 assign ready=phase==IDLE || phase==DONE || phase==ABORT;
 assign motor_enable=rst_n && moving && !stop;
 wire reverse=(phase==BACK || phase==LEFT_RETRACE || phase==RIGHT_RETRACE);
 wire signed [31:0] step_l=reverse ? -count_l:count_l;
 wire signed [31:0] step_r=reverse ? -count_r:count_r;
 wire signed [32:0] add_l=$signed({1'b0,progress_l})+$signed(step_l);
 wire signed [32:0] add_r=$signed({1'b0,progress_r})+$signed(step_r);
 wire [32:0] progress_sum={1'b0,progress_l}+{1'b0,progress_r};
 wire wrong_speed_l=reverse ? speed_l>8:speed_l< -8;
 wire wrong_speed_r=reverse ? speed_r>8:speed_r< -8;
 reg [31:0] desired_rpm;
 always @* begin
   // Each arc phase first holds its steering for WAIT_MS, then enables the wheels.
   // Retrace phases keep the same steering angle while reversing the arc.
   steering=(phase==LEFT || phase==LEFT_RETRACE) ? 1:
         (phase==RIGHT && elapsed>=WAIT_MS) ? 2:
         (phase==RIGHT_RETRACE) ? 2:0;
   desired_rpm=MAX_RPM;
   if(progress_sum+2*SLOW_COUNTS>=2*GOAL)desired_rpm=SLOW_RPM;
   target_rpm=reverse ? -$signed(rpm_mag):$signed(rpm_mag);
   if(!motor_enable)target_rpm=0;
 end
 task enter;
   input [3:0] next_phase;
   begin
     phase<=next_phase;elapsed<=0;ramp_ms<=0;rpm_mag<=10;
     progress_l<=0;progress_r<=0;zero_l<=0;zero_r<=0;wrong_l<=0;wrong_r<=0;
     fresh_l<=0;fresh_r<=0;
     balance_l<=0;balance_r<=0;imbalance_ms<=0;
   end
 endtask
 task fail;
   input [3:0] code;
   begin
     // Do not call enter(): it erased the evidence needed to diagnose fault9.
     phase<=FAULT;fault_code<=code;fault_phase<=phase;
     fault_progress_left<=progress_l;fault_progress_right<=progress_r;
     // Override any same-edge accumulation: keep exactly the evaluated values.
     progress_l<=progress_l;progress_r<=progress_r;
   end
 endtask
 always @(posedge clk or negedge rst_n) begin
   if(!rst_n)begin
     phase<=IDLE;fault_code<=0;divider<=0;elapsed<=0;ramp_ms<=0;rpm_mag<=0;
     progress_l<=0;progress_r<=0;zero_l<=0;zero_r<=0;wrong_l<=0;wrong_r<=0;
     fresh_l<=0;fresh_r<=0;
     balance_l<=0;balance_r<=0;imbalance_ms<=0;
     fault_phase<=0;fault_progress_left<=0;fault_progress_right<=0;
   end else begin
     divider<=tick ? 0:divider+1'b1;
     if(tick && busy)elapsed<=elapsed+1'b1;
     // Registered products avoid a long multiply/subtract/compare control path.
     balance_l<=$signed({1'b0,progress_l})*RIGHT_TRIM_PERCENT;
     balance_r<=$signed({1'b0,progress_r})*LEFT_TRIM_PERCENT;
     if(moving)begin
       if(valid_l)begin
         progress_l<=add_l<0 ? 0:add_l[31:0];fresh_l<=0;
         zero_l<=count_l==0 ? zero_l+1'b1:0;
         wrong_l<=wrong_speed_l ? wrong_l+1'b1:0;
       end else if(tick && fresh_l<=ENCODER_STALE_MS)fresh_l<=fresh_l+1'b1;
       if(valid_r)begin
         progress_r<=add_r<0 ? 0:add_r[31:0];fresh_r<=0;
         zero_r<=count_r==0 ? zero_r+1'b1:0;
         wrong_r<=wrong_speed_r ? wrong_r+1'b1:0;
       end else if(tick && fresh_r<=ENCODER_STALE_MS)fresh_r<=fresh_r+1'b1;
       if(tick)begin
         if(rpm_mag>desired_rpm)rpm_mag<=desired_rpm;
         else if(ramp_ms==99)begin
           ramp_ms<=0;rpm_mag<=rpm_mag+5>=desired_rpm ? desired_rpm:rpm_mag+5;
         end else ramp_ms<=ramp_ms+1'b1;
       end
     end
     case(phase)
       IDLE,DONE,ABORT:if(start)begin
         enter(PREP);fault_code<=0;fault_phase<=0;fault_progress_left<=0;fault_progress_right<=0;
       end
       PREP:if(elapsed>=WAIT_MS)enter(FWD);
       FWD:if(progress_sum>=2*GOAL)enter(WAIT_BACK);
       WAIT_BACK:if(elapsed>=WAIT_MS)enter(BACK);
       BACK:if(progress_sum>=2*GOAL)enter(LEFT);
       LEFT:if(elapsed>=WAIT_MS && progress_sum>=2*GOAL)enter(LEFT_RETRACE);
       LEFT_RETRACE:if(elapsed>=WAIT_MS && progress_sum>=2*GOAL)enter(RIGHT);
       RIGHT:if(elapsed>=2*WAIT_MS && progress_sum>=2*GOAL)enter(RIGHT_RETRACE);
       RIGHT_RETRACE:if(elapsed>=WAIT_MS && progress_sum>=2*GOAL)enter(FINISH);
       FINISH:if(elapsed>=WAIT_MS)enter(DONE);
       default:begin end
     endcase
     if(moving)begin
       // Encoder windows are 10ms, as in the preserved motor controller.
       if(zero_l>=NO_FEEDBACK_MS/10 || zero_r>=NO_FEEDBACK_MS/10)fail(3);
       if(wrong_l>=10 || wrong_r>=10)fail(4);
       if(fresh_l>=ENCODER_STALE_MS || fresh_r>=ENCODER_STALE_MS)fail(5);
       if(elapsed>=SEGMENT_TIMEOUT_MS)fail(6);
       // Ignore intentional 5% trim, permit launch transient, require persistence.
       // Do not apply straight-line wheel balance checks on an Ackermann arc.
       if(phase==FWD || phase==BACK)begin
         if(tick)begin
           if(elapsed<IMBALANCE_GRACE_MS || !balance_bad)imbalance_ms<=0;
           else if(imbalance_ms<IMBALANCE_HOLD_MS)imbalance_ms<=imbalance_ms+1'b1;
         end
         if(imbalance_ms>=IMBALANCE_HOLD_MS)fail(9);
       end
     end
     if(stop && busy)enter(ABORT);
   end
 end
endmodule
