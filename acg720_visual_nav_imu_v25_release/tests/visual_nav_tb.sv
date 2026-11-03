`timescale 1ns/1ps
module visual_nav_tb;
 reg clk=0;always #5 clk=~clk;
 reg imu_ok=1,imu_feed=1;reg signed [31:0] yaw=0;reg iu=0;
 reg rst=0,start=0,stop=0,live=0,update=0,bv=0,configured=1;
 reg [11:0] x0=300,y0=100,x1=340,y1=300;
 reg [2:0] tv=7;
 reg [15:0] dl=1000,df=1000,dr=1000;
 reg signed [31:0] cl=1,cr=1,sl=10,sr=10;
 reg vl=0,vr=0,feed=1,frames=1;
 wire en,busy,done,fault,ready,can_start,brake;
 wire signed [31:0] rpm;wire [15:0] pulse,fl,fr,age;
 wire [1:0] steer;wire [3:0] phase,code,fp;
 wire [31:0] pl,pr,elapsed,fpl,fpr;wire [11:0] cx;wire signed [12:0] err;
 visual_approach_controller #(.CLK_HZ(100000),.WAIT_MS(30),.VISION_LATENCY_MS(20),
   .TARGET_STALE_MS(50),.ENCODER_STALE_MS(40),.NO_FEEDBACK_MS(200),
   .MAX_TIME_MS(2000),.MAX_DISTANCE_MM(30),.BRAKE_MS(10)) dut(
   clk,rst,start,stop,live,update,bv,yaw,imu_ok,iu,x0,y0,x1,y1,configured,tv,dl,df,dr,
   cl,cr,sl,sr,vl,vr,en,rpm,pulse,steer,phase,code,busy,done,fault,ready,can_start,brake,
   pl,pr,elapsed,fl,fr,age,cx,err,fp,fpl,fpr,,,,,,);
 integer cycles=0;
 always @(negedge clk)begin
   cycles=cycles+1;vl=0;vr=0;update=0;iu=0;
   if(cycles%1000==0 && imu_feed)iu=1;
   if(cycles%1000==0 && feed)begin vl=1;vr=1;end
   if(cycles%1000==100 && frames)update=1;
 end
 task ms(input integer n);repeat(n*100)@(negedge clk);endtask
 task boot;
 begin
   rst=0;imu_ok=1;imu_feed=1;yaw=0;live=0;bv=0;df=1000;tv=7;dl=1000;dr=1000;stop=0;feed=1;frames=1;
   cl=1;cr=1;sl=10;sr=10;x0=300;x1=340;
   ms(1);rst=1;ms(1);
 end endtask
 task acquire;
 begin live=1;bv=1;ms(40);if(!can_start)$fatal(1,"Target not acquired");end endtask
 task launch;
 begin @(posedge clk);#1;start=1;@(posedge clk);#1;start=0;ms(32);if(!en)$fatal(1,"Did not launch: %d",code);end endtask
 task fault_is(input integer reason);
 begin ms(12);if(!fault || en || code!=reason)$fatal(1,"Expected fault %d got %d phase%d en%b",reason,code,phase,en);end endtask
 initial begin
   boot();start=1;ms(2);start=0;if(busy)$fatal(1,"Started without target");
   acquire();launch();
   // Left-to-right changes are gradual, staying below the target-jump veto.
   x0=200;x1=240;ms(120);if(pulse>=1400 || steer!=1)$fatal(1,"Left target wrong steering");
   x0=290;x1=330;ms(40);x0=380;x1=420;ms(250);
   if(pulse<=1400 || steer!=2)$fatal(1,"Right target wrong steering");
   x0=300;x1=340;ms(250);df=220;ms(2);if(rpm>10)$fatal(1,"No creep speed");
   df=100;ms(1);if(!done || en || !brake)$fatal(1,"No arrival stop");
   df=500;ms(30);if(en || busy || brake)$fatal(1,"Auto restarted / brake unbounded");
   boot();acquire();launch();bv=0;fault_is(1);
   bv=1;ms(40);launch();live=0;fault_is(1);
   boot();acquire();launch();frames=0;ms(60);fault_is(1);
   boot();acquire();launch();tv=5;fault_is(2);
   boot();live=1;bv=1;tv=5;ms(40);start=1;ms(2);start=0;
   if(en || busy || can_start)$fatal(1,"Started without front range");
   boot();acquire();launch();x0=500;x1=540;fault_is(1);
   boot();acquire();launch();x0=210;x1=250;ms(20);df=100;fault_is(8);
   boot();acquire();launch();dl=100;fault_is(7);
   boot();acquire();launch();feed=0;ms(50);fault_is(5);
   boot();acquire();launch();cl=0;ms(220);fault_is(3);
   boot();acquire();launch();sl=-15;ms(110);fault_is(4);
   boot();acquire();launch();cl=100;cr=100;ms(40);fault_is(9);
   boot();acquire();launch();imu_ok=0;fault_is(10);
   boot();acquire();launch();imu_feed=0;ms(105);fault_is(10);
   boot();acquire();launch();stop=1;ms(1);if(en || brake || phase!=12)$fatal(1,"S3 priority failed");
   boot();acquire();launch();ms(2010);fault_is(6);
   $display("PASS visual_nav: acquisition, signed steering, creep, 100mm arrival, latches, restart, vision/TOF/IMU/encoder/watchdogs/limits/S3");$finish;
 end
 initial begin #20000000;$fatal(1,"Timeout");end
endmodule
