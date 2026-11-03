`timescale 1ns/1ps
module imu_heading_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,link=1,sample=0;
 reg signed [15:0] gz=0;
 wire signed [31:0] yaw,roll,pitch;
 wire calibrated,valid,update;
 imu_attitude #(.CAL_LOG2(3)) dut(.clk(clk),.rst_n(rst),.recalibrate(1'b0),.link_ok(link),.sample_valid(sample),
   .ax(16'sd0),.ay(16'sd0),.az(16'sd8192),.gx(16'sd0),.gy(16'sd0),.gz(gz),.dt_us(16'd10000),
   .roll(roll),.pitch(pitch),.yaw(yaw),.calibrated(calibrated),.attitude_valid(valid),.update(update),
   .cal_count(),.accel_trusted(),.tilt_warning(),.overrun());
 task frame;
   begin @(negedge clk);sample=1;@(negedge clk);sample=0;repeat(150)@(negedge clk);end
 endtask
 integer i;
 initial begin
   repeat(3)@(negedge clk);rst=1;
   for(i=0;i<8;i=i+1)frame();
   if(!calibrated || !valid || yaw!=0)$fatal(1,"stationary gyro calibration failed");
   gz=6550;for(i=0;i<90;i=i+1)frame();
   if(yaw<89*65536 || yaw>91*65536)$fatal(1,"gyro yaw scale wrong %d",yaw);
   gz=-6550;for(i=0;i<90;i=i+1)frame();
   if(yaw>65536 || yaw< -65536)$fatal(1,"gyro yaw reverse wrong");
   link=0;repeat(3)@(negedge clk);
   if(valid || calibrated || yaw!=0)$fatal(1,"link loss did not invalidate heading");
   $display("PASS: real attitude pipeline stationary calibration, gyro sign/scale, 90deg rotation/return, link-loss invalidation");$finish;
 end
endmodule
