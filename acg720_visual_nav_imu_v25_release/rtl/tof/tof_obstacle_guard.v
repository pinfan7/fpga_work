`timescale 1ns/1ps
// Every sensor has veto authority. Recovery NEVER resumes motion by itself.
module tof_obstacle_guard #(
 parameter integer CLK_HZ=50000000,SLOW_MM=700,STOP_MM=350,
 parameter integer SLOW_CLEAR_MM=750,RESTART_MM=400,
 parameter integer CLEAR_MS=300,BRAKE_MS=300
)(
 input clk,rst_n,configured,
 input [15:0] distance_l,distance_f,distance_r,
 input [2:0] valid,
 input running,moving,start_accepted,manual_stop,
 output [15:0] min_distance,
 output all_valid,can_start,halt,
 output reg slow,latched,
 output reg [3:0] reason,
 output brake
);
 wire [15:0] min_lf=distance_l<distance_f ? distance_l:distance_f;
 assign min_distance=min_lf<distance_r ? min_lf:distance_r;
 assign all_valid=configured && (&valid);
 wire [2:0] near_mask={distance_r<=STOP_MM,distance_f<=STOP_MM,distance_l<=STOP_MM};
 wire unsafe_now=!all_valid || (|near_mask);
 reg [31:0] divider,clear_ms,brake_ms;
 wire tick=divider==CLK_HZ/1000-1;
 assign can_start=all_valid && min_distance>=RESTART_MM && clear_ms>=CLEAR_MS;
 assign halt=latched || (running && unsafe_now);
 assign brake=rst_n && brake_ms!=0 && !manual_stop;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
   divider<=0;clear_ms<=0;brake_ms<=0;slow<=0;latched<=0;reason<=0;
 end else begin
   divider<=tick ? 0:divider+1'b1;
   if(!all_valid || min_distance<RESTART_MM)clear_ms<=0;
   else if(tick && clear_ms<CLEAR_MS)clear_ms<=clear_ms+1'b1;
   if(!all_valid || min_distance<=SLOW_MM)slow<=1;
   else if(min_distance>=SLOW_CLEAR_MM)slow<=0;
   if(tick && brake_ms!=0)brake_ms<=brake_ms-1'b1;
   if(start_accepted && can_start)begin latched<=0;reason<=0;end
   if(running && unsafe_now && !latched)begin
     latched<=1;
     // bit3 = stale/invalid; bit2/1/0 = right/front/left too close.
     reason<=all_valid ? {1'b0,near_mask}:4'b1000;
     if(moving && !manual_stop)brake_ms<=BRAKE_MS;
   end
   if(manual_stop)brake_ms<=0;
 end
endmodule
