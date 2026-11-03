`timescale 1ns/1ps
module nav_cdc_tb;
 reg src=0,dst=0,rst=0,vs=0,valid=0,commit=0,enabled=0;
 reg src_run=1;
 always #7 if(src_run)src=~src;
 always #5 dst=~dst;
 reg [11:0] x0=0,y0=0,x1=0,y1=0;
 wire update,v,live;wire [11:0] a,b,c,d;
 vision_target_cdc dut(src,dst,rst,vs,valid,x0,y0,x1,y1,update,v,a,b,c,d);
 camera_frame_watchdog #(.CLK_HZ(100000),.TIMEOUT_MS(5)) watch(dst,rst,commit,enabled,live);
 integer packets=0;reg expect_valid;reg [47:0] expected;
 always @(posedge dst)if(update)begin
   packets=packets+1;
   if({a,b,c,d}!==expected || v!==expect_valid)$fatal(1,"Torn/wrong mailbox payload");
 end
 task frame(input bit vv,input integer n);
 begin
   @(negedge src);valid=vv;x0=n;y0=n+10;x1=n+30;y1=n+100;
   expected={x0,y0,x1,y1};expect_valid=vv;
   vs=1;repeat(3)@(negedge src);vs=0;
   repeat(12)@(negedge src);
 end endtask
 initial begin
   #1;rst=0;#40;rst=1;
   frame(1,20);frame(1,120);frame(0,80);
   if(packets!=3)$fatal(1,"Missing valid/invalid frame");
   enabled=1;repeat(5)@(posedge dst);commit=~commit;
   repeat(8)@(posedge dst);if(!live)$fatal(1,"No fresh frame");
   // A running display clock without camera commits must expire.
   repeat(520)@(posedge dst);if(live)$fatal(1,"Frozen camera kept live");
   commit=~commit;repeat(8)@(posedge dst);if(!live)$fatal(1,"Fresh frame not recovered");
   enabled=0;repeat(8)@(posedge dst);if(live)$fatal(1,"Pattern mode still live");
   src_run=0;repeat(30)@(posedge dst);
   if(packets!=3)$fatal(1,"Spurious packet on paused source clock");
   $display("PASS nav_cdc: atomic boxes, invalid message, frozen-frame expiry, source disable");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
