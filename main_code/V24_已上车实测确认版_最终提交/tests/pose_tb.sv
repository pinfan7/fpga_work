`timescale 1ns/1ps
module pose_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,clear=0,track=1,vl=0,vr=0;
 reg signed [31:0] yaw=0,l=0,r=0;
 wire signed [31:0] x,y,h;wire updated,fault;
 nav_pose dut(.clk(clk),.rst_n(rst),.clear(clear),.track(track),.yaw_q16(yaw),
 .count_l(l),.count_r(r),.valid_l(vl),.valid_r(vr),.x_q8(x),.y_q8(y),.heading_q16(h),.updated(updated),.fault(fault));
 task origin;
   input integer degrees;
   begin @(negedge clk);yaw=degrees*65536;clear=1;
     @(negedge clk);clear=0;repeat(2)@(negedge clk);end
 endtask
 task sample;
   input integer a,b;
   begin @(negedge clk);l=a;r=b;vl=1;vr=1;
     @(negedge clk);vl=0;vr=0;repeat(5)@(negedge clk);end
 endtask
 integer i;
 initial begin
   repeat(3)@(negedge clk);rst=1;origin(0);
   for(i=0;i<10;i=i+1)sample(100,100);
   if(x<33400 || x>33700 || y!=0 || fault)$fatal(1,"straight distance scale wrong x=%d y=%d",x,y);
   origin(0);yaw=30*65536;repeat(3)@(negedge clk);sample(100,100);
   if(x<2890 || x>2920 || y<1660 || y>1690)$fatal(1,"30 degree projection wrong x=%d y=%d",x,y);
   origin(0);yaw=-90*65536;repeat(3)@(negedge clk);sample(100,100);
   if(x!=0 || y> -3340 || y< -3370)$fatal(1,"right direction wrong");
   origin(179);yaw=-179*65536;repeat(3)@(negedge clk);if(h!=2*65536)$fatal(1,"yaw wrap wrong %d",h);
   origin(0);track=0;sample(100,100);if(x!=0 || y!=0)$fatal(1,"disabled pose moved");
   track=1;sample(3000,3000);if(!fault)$fatal(1,"encoder spike was accepted");
   $display("PASS: pose distance; 30/-90 degree projection; heading wrap; origin reset; disabled tracking; count spike");$finish;
 end
endmodule
