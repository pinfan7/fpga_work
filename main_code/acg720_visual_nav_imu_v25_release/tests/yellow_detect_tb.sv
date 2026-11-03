`timescale 1ns/1ps
module yellow_detect_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,de=0,vs=0;reg [23:0] rgb=0;
 wire valid;wire [11:0] x0,y0,x1,y1;
 // Exercise the production 640x480 detector with unchanged V21 defaults.
 yellow_cup_detector dut(.clk(clk),.rst_n(rst),.enable(1'b1),
   .rgb(rgb),.de(de),.vs(vs),.box_valid(valid),
   .box_x0(x0),.box_y0(y0),.box_x1(x1),.box_y1(y1));
 task frame(input integer mode);
 integer x,y;begin
   @(negedge clk);vs=1;de=0;
   repeat(4)@(negedge clk);vs=0;
   for(y=0;y<480;y=y+1)begin
     for(x=0;x<640;x=x+1)begin
       @(negedge clk);de=1;
       rgb=(mode==2 || (mode==1 && x>=160 && x<240 && y>=160 && y<320)) ?
         24'hdcc820:24'h202020;
     end
     @(negedge clk);de=0;repeat(8)@(negedge clk);
   end
   repeat(40000)@(negedge clk);
 end endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;
   repeat(5)frame(1);
   if(!valid || x0>168 || x1<232 || y0>168 || y1<312)
     $fatal(1,"Yellow bounding box incorrect valid%b (%d,%d)-(%d,%d)",valid,x0,y0,x1,y1);
   repeat(3)frame(0);if(valid)$fatal(1,"Target remained after yellow disappeared");
   repeat(3)frame(2);if(valid)$fatal(1,"Whole-screen yellow accepted as compact target");
   $display("PASS yellow_detect: unchanged production RGB/shape thresholds, actual 640x480 yellow box, disappearance, whole-screen rejection");
   $finish;
 end
endmodule
