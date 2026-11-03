`timescale 1ns/1ps
// Preserve the existing data-loss stop; no distance-only startup/stop gate.
module tof_obstacle_guard #(
 parameter integer CLK_HZ=50000000,
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
 wire unsafe_now=!all_valid;
 reg [31:0] divider,clear_ms,brake_ms;
 wire tick=divider==CLK_HZ/1000-1;
 assign can_start=all_valid && clear_ms>=CLEAR_MS;
 assign halt=latched || (running && unsafe_now);
 assign brake=rst_n && brake_ms!=0 && !manual_stop;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin
   divider<=0;clear_ms<=0;brake_ms<=0;slow<=0;latched<=0;reason<=0;
 end else begin
   divider<=tick ? 0:divider+1'b1;
   if(!all_valid)clear_ms<=0;
   else if(tick && clear_ms<CLEAR_MS)clear_ms<=clear_ms+1'b1;
   // Distance-based speed limiting is handled only by the approach controller
   // at its 150mm bypass trigger. Keep this module for TOF data-loss locking.
   slow<=0;
   if(tick && brake_ms!=0)brake_ms<=brake_ms-1'b1;
   if(start_accepted && can_start)begin latched<=0;reason<=0;end
   if(running && unsafe_now && !latched)begin
     latched<=1;
     // bit3 = stale/invalid TOF data.
     reason<=4'b1000;
     if(moving && !manual_stop)brake_ms<=BRAKE_MS;
   end
   if(manual_stop)brake_ms<=0;
 end
endmodule
