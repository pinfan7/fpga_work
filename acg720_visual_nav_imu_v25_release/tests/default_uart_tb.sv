`timescale 1ns/1ps
module default_uart_tb;
 reg clk=0;always #10 clk=~clk;
 reg rst=0,rx=1;
 wire[15:0] distance_mm,frames;
 wire[7:0] status;
 wire valid,fault;
 tof_channel #(.TIMEOUT_MS(2)) dut(clk,rst,rx,distance_mm,status,valid,fault,frames);
 task tick(input integer n);repeat(n) @(negedge clk);endtask
 task send_byte(input[7:0] b,input integer clocks);
 integer i;begin
  rx=0;tick(clocks);
  for(i=0;i<8;i=i+1) begin rx=b[i];tick(clocks);end
  rx=1;tick(clocks);
 end endtask
 task frame(input integer clocks);
 begin
  send_byte('h5a,clocks);send_byte('h5a,clocks);send_byte('h15,clocks);send_byte(3,clocks);
  send_byte(0,clocks);send_byte('hfa,clocks);send_byte(1,clocks);send_byte('hc7,clocks);
  tick(100);
 end endtask
 initial begin
  tick(10);rst=1;tick(10);
  // Reject a short low glitch, then tolerate approximately +/-2% baud error.
  rx=0;tick(100);rx=1;tick(6000);
  frame(5104);
  if(!valid || distance_mm!=250 || status!=1 || frames!=1) $fatal(1,"50MHz faster UART failed");
  frame(5312);
  if(!valid || distance_mm!=250 || frames!=2 || fault) $fatal(1,"50MHz slower UART failed");
  tick(100100);if(valid) $fatal(1,"Real clock timeout failed");
  $display("PASS: 50MHz / 9600 baud, start glitch rejection, +/-2 percent sender baud tolerance, timeout");
  $finish;
 end
 initial begin #30000000;$fatal(1,"Watchdog");end
endmodule
