`timescale 1ns/1ps
module noimu_sequence_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,stop=0,valid_l=0,valid_r=0;
 reg signed [31:0] count_l=0,count_r=0,speed_l=0,speed_r=0;
 wire enable;wire signed [31:0] target;wire [1:0] steering;
 wire [3:0] phase,code;wire busy,done,fault,ready;
 // Only the clock is accelerated; distance conversion and 10ms windows are real.
 auto_sequence #(.CLK_HZ(10000),.WAIT_MS(5)) dut(
 .clk(clk),.rst_n(rst_n),.start(start),.stop(stop),
 .count_l(count_l),.count_r(count_r),.speed_l(speed_l),.speed_r(speed_r),
 .valid_l(valid_l),.valid_r(valid_r),.motor_enable(enable),.target_rpm(target),
 .steering(steering),.phase(phase),.fault_code(code),.busy(busy),.done(done),.fault(fault),.ready(ready));
 integer model=0,n=0,last_phase=0,segments=0,pl,pr,samples=0;
 integer right_center_checks=0,right_settle_checks=0,right_move_checks=0;
 // Unequal wheel counts remain independent; 1m is their average, not either wheel.
 always @(negedge clk)begin
   valid_l=0;valid_r=0;
   if(enable && model!=2)begin
     if(n==99)begin
       n=0;valid_l=1;valid_r=1;
       if(model==1)begin count_l=0;count_r=0;speed_l=0;speed_r=0;end
       else begin
         count_l=(phase==4 || phase==6 || phase==8) ? -41:41;count_r=(phase==4 || phase==6 || phase==8) ? -39:39;
         speed_l=(phase==4 || phase==6 || phase==8) ? -60:60;speed_r=(phase==4 || phase==6 || phase==8) ? -60:60;
       end
     end else n=n+1;
   end else n=0;
 end
 task reset_test;
 begin rst_n=0;start=0;stop=0;repeat(5)@(negedge clk);rst_n=1;repeat(5)@(negedge clk);end
 endtask
 task launch;
 begin @(negedge clk);start=1;@(negedge clk);start=0;end
 endtask
 task check;
 input ok;input [255:0] message;
 begin if(!ok)$fatal(1,"%s",message);end
 endtask
 // Check path endpoint and independent accumulators on every completed segment.
 always @(posedge clk)begin
   pl=dut.progress_l;pr=dut.progress_r;last_phase=phase;
   #1;
   if(rst_n && model==0 && (last_phase==2 || last_phase==4 || last_phase==5 || last_phase==6 || last_phase==7 || last_phase==8)
      && phase!=last_phase && phase!=11 && phase!=12)begin
     check(!fault,"unexpected mission fault");
     check(pl+pr>=2*dut.GOAL,"segment ended before 1m average");
     check(pl>pr,"left/right counts were not independent");
     check(pl+pr<2*dut.GOAL+160,"excess segment overshoot");segments=segments+1;
   end
   if(rst_n && enable)begin
     check(phase==2 || phase==4 || phase==5 || phase==6 || phase==7 || phase==8,"enabled outside movement");
     if(phase==4 || phase==6 || phase==8)check(target<0,"reverse target sign");
     else check(target>0,"forward target sign");
   end
   if(rst_n && (phase==5 || phase==6))check(steering==1,"left steering/retrace");
   if(rst_n && phase==7)begin
     if(dut.elapsed<dut.WAIT_MS)begin
       check(steering==0 && !enable && target==0,"right entry must stop and center");
       if(model==0)right_center_checks=right_center_checks+1;
     end else if(dut.elapsed<2*dut.WAIT_MS)begin
       check(steering==2 && !enable && target==0,"right steering must settle before motion");
       if(model==0)right_settle_checks=right_settle_checks+1;
     end else begin
       check(steering==2 && enable,"right motion requires settled steering");
       if(model==0)right_move_checks=right_move_checks+1;
     end
   end
   if(rst_n && phase==8)check(steering==2,"right retrace must keep right steering");
 end
 initial begin
   reset_test;check(ready,"no-IMU idle was not ready");launch;
   wait(done || fault);#2;check(done && !fault && segments==6,"full mission failed");
   check(right_center_checks>0 && right_settle_checks>0 && right_move_checks>0,
         "right center/settle/motion coverage missing");
   check(!enable && steering==0,"completion did not stop/center");
   reset_test;launch;wait(enable);@(negedge clk);stop=1;#1;
   check(!enable,"stop must inhibit immediately");repeat(3)@(negedge clk);
   check(phase==12 && !fault,"S3-equivalent abort");
   model=1;reset_test;launch;wait(fault);#2;check(code==3 && !enable,"missing count protection");
   model=2;reset_test;launch;wait(fault);#2;check(code==5 && !enable,"missing update protection");
   $display("PASS: six 1m segments, both arc retraces, right center/settle/motion, independent counts, abort and feedback faults");
   $finish;
 end
 initial begin #20000000;$fatal(1,"test timeout");end
endmodule
