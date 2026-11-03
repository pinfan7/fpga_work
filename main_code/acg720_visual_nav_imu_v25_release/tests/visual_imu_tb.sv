`timescale 1ns/1ps
module visual_imu_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,tick=0,ok=1,iu=0,obs=0;
 reg signed [31:0] yaw=0;
 reg [11:0] cx=320;
 wire sensor_ready,ready;wire [15:0] pulse;
 wire signed [31:0] heading,goal,error,rate;
 visual_imu_heading dut(.clk(clk),.rst_n(rst),.tick_ms(tick),.imu_ok(ok),.imu_update(iu),
   .yaw_q16(yaw),.observation(obs),.center_x(cx),.sensor_ready(sensor_ready),
   .heading_ready(ready),.request_us(pulse),.heading_q8(heading),.goal_q8(goal),
   .error_q8(error),.rate_q8(rate));
 task step(input integer n);
 integer k;begin
   for(k=0;k<n;k=k+1)begin
     @(negedge clk);tick=1;iu=1;
     @(negedge clk);tick=0;iu=0;
     repeat(20)@(negedge clk);
   end
 end endtask
 task observe(input integer x);
 begin @(negedge clk);cx=x;obs=1;@(negedge clk);obs=0;repeat(10)@(negedge clk);end
 endtask
 task boot;
 begin rst=0;ok=1;yaw=0;obs=0;step(2);rst=1;step(125);end
 endtask
 integer i,plant_q8,target_q8,delta_q8;
 integer hist[0:99];
 initial begin
   boot();if(ready)$fatal(1,"Ready without visual goal");
   observe(160);if(!ready || pulse>=1400 || goal!=10*256)$fatal(1,"Left visual bearing wrong");
   // Physical rotation changes steering even without a new camera frame.
   yaw=10*65536;step(200);
   if(error!=0 || pulse<1395 || pulse>1405)$fatal(1,"IMU did not remove completed turn %d",pulse);
   yaw=15*65536;step(80);if(pulse<=1400)$fatal(1,"Overshoot did not countersteer");
   ok=0;step(1);if(ready || sensor_ready || pulse!=1400)$fatal(1,"IMU fault not rejected");
   boot();yaw=179*65536;step(120);observe(288);
   if(goal!= -179*256 || error!=2*256)$fatal(1,"Wrap +180 failed goal%d err%d",goal,error);
   yaw=-179*65536;step(100);observe(352);
   if(goal!=179*256 || error!= -2*256)$fatal(1,"Wrap -180 failed");
   boot();yaw=10*65536;step(20);observe(320);
   if(goal!=0 || error!= -10*256)$fatal(1,"Delayed image used current instead of historical yaw");
   boot();observe(320);
   // Updates freeze while link remains asserted: separate freshness watchdog.
   for(i=0;i<102;i=i+1)begin
     @(negedge clk);tick=1;@(negedge clk);tick=0;repeat(20)@(negedge clk);
   end
   if(ready || sensor_ready)$fatal(1,"Frozen heading accepted");
   boot();observe(320);yaw=-90*65536;step(1);
   if(pulse!=1039)$fatal(1,"Left mechanical clamp not enforced %d",pulse);
   yaw=90*65536;step(1);
   if(pulse!=1850)$fatal(1,"Right mechanical clamp not enforced %d",pulse);
   // Closed-loop yaw plant with a fixed world target and 100ms old images.
   // 1us command produces 1.25deg/s yaw rate in this deliberately simple model.
   boot();plant_q8=0;target_q8=12*256;
   for(i=0;i<100;i=i+1)hist[i]=0;
   for(i=0;i<3000;i=i+1)begin
     if(i%20==0)observe(320-(target_q8-hist[i%100])/16);
     delta_q8=(1400-$signed({1'b0,pulse}))*256/800;
     hist[i%100]=plant_q8;plant_q8=plant_q8+delta_q8;
     yaw=plant_q8*256;step(1);
     if(pulse<1039 || pulse>1850 || plant_q8< -5*256 || plant_q8>25*256)
       $fatal(1,"Closed-loop plant diverged yaw%d pulse%d",plant_q8,pulse);
   end
   if(plant_q8<9*256 || plant_q8>15*256)$fatal(1,"Plant failed to converge yaw%d",plant_q8);
   $display("PASS visual_imu: bearing, real yaw correction, overshoot damping, wrap, image latency, fault/freshness, mechanical limits, delayed-image yaw plant");
   $finish;
 end
endmodule
