`timescale 1ns/1ps
module tof3_tb;
 reg clk=0;always #500 clk=~clk; // 1 MHz, accelerated UART: 100 kbaud
 reg rst=0,lrx=1,frx=1,rrx=1;
 reg k1=1,k2=1,k3=1,k4=1;
 wire ltx,ftx,rtx,dbg;
 wire [7:0] safe,led;
 tof3_test_top #(.CLK_HZ(1000000),.TOF_BAUD(100000),.DEBUG_BAUD(100000),
  .TIMEOUT_MS(20),.INIT_DELAY_MS(2)) dut(
  clk,rst,k1,k2,k3,k4,lrx,frx,rrx,ltx,ftx,rtx,dbg,
  safe[7],safe[6],safe[5],safe[4],safe[3],safe[2],safe[1],safe[0],led);
 task tick(input integer n);repeat(n) @(negedge clk);endtask
 task byte_l(input [7:0] b);
  integer i;begin lrx=0;tick(10);for(i=0;i<8;i=i+1) begin lrx=b[i];tick(10);end lrx=1;tick(10);end
 endtask
 task byte_f(input [7:0] b);
  integer i;begin frx=0;tick(10);for(i=0;i<8;i=i+1) begin frx=b[i];tick(10);end frx=1;tick(10);end
 endtask
 task byte_r(input [7:0] b);
  integer i;begin rrx=0;tick(10);for(i=0;i<8;i=i+1) begin rrx=b[i];tick(10);end rrx=1;tick(10);end
 endtask
 task frame_l(input [15:0] d,input [7:0] s,input corrupt);
  reg[7:0] sum;begin sum=8'hcc+d[15:8]+d[7:0]+s;
   byte_l('h5a);byte_l('h5a);byte_l('h15);byte_l(3);byte_l(d[15:8]);byte_l(d[7:0]);byte_l(s);byte_l(sum^corrupt);tick(10);end
 endtask
 task frame_f(input [15:0] d);
  reg[7:0] sum;begin sum=8'hcc+d[15:8]+d[7:0]+1;
   byte_f('h5a);byte_f('h5a);byte_f('h15);byte_f(3);byte_f(d[15:8]);byte_f(d[7:0]);byte_f(1);byte_f(sum);tick(10);end
 endtask
 task frame_r(input [15:0] d);
  reg[7:0] sum;begin sum=8'hcc+d[15:8]+d[7:0]+1;
   byte_r('h5a);byte_r('h5a);byte_r('h15);byte_r(3);byte_r(d[15:8]);byte_r(d[7:0]);byte_r(1);byte_r(sum);tick(10);end
 endtask
 // Independent command decoder verifies actual serialized output, not index only.
 wire [7:0] config_data;wire config_ready,config_error;
 uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) monitor(clk,rst,ltx,config_data,config_ready,config_error);
 integer command_count=0;
 reg[7:0] expected[0:8];
 always @(posedge clk) if(rst) begin
  if(safe!==0) $fatal(1,"Drive output became active");
  if(ltx!==ftx || ltx!==rtx) $fatal(1,"Config outputs differ");
  if(config_error) $fatal(1,"Config UART framing error");
  if(config_ready) begin
   if(command_count>=9) $fatal(1,"Unexpected extra command");
   if(config_data!==expected[command_count]) $fatal(1,"Config command mismatch");
   command_count=command_count+1;
  end
 end
 wire[7:0] debug_data;wire debug_ready,debug_error;
 uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) debug_monitor(clk,rst,dbg,debug_data,debug_ready,debug_error);
 integer chars=0,lines=0;
 always @(posedge clk) if(rst && debug_ready) begin
  if(chars==0 && debug_data!="L") $fatal(1,"Telemetry header");
  if(chars==42 && debug_data!=13) $fatal(1,"Telemetry CR");
  if(chars==43) begin
   if(debug_data!=10) $fatal(1,"Telemetry LF");
   chars=0;lines=lines+1;
  end else chars=chars+1;
 end
 initial begin
  expected[0]='ha5;expected[1]='h51;expected[2]='hf6;
  expected[3]='ha5;expected[4]='h64;expected[5]='h09;
  expected[6]='ha5;expected[7]='h45;expected[8]='hea;
  tick(10);rst=1;tick(10);
  if(dut.vl || dut.vf || dut.vr) $fatal(1,"Valid without frame");
  // Junk and triple header recovery; channels operate concurrently.
  byte_l('h12);byte_l('h5a);
  fork frame_l(250,1,0);frame_f(1000);frame_r(200);join
  tick(5);
  if(!dut.vl || !dut.vf || !dut.vr || dut.dl!=250 || dut.df!=1000 || dut.dr!=200) $fatal(1,"Concurrent receive");
  if(led[5:0]!==6'b101111) $fatal(1,"Status/near LEDs");
  k1=0;tick(10);if(led!=25) $fatal(1,"Selected distance cm");k1=1;tick(10);
  frame_l(500,1,1);if(dut.vl || !dut.el || dut.dl!=250) $fatal(1,"Bad checksum was not invalidated or corrupted saved distance");
  tick(21000);if(dut.vl) $fatal(1,"Bad checksum extended freshness");
  frame_l(500,'h21,0);if(dut.vl || dut.sl!='h21) $fatal(1,"Invalid range status");
  frame_l(0,1,0);if(dut.vl) $fatal(1,"Zero range accepted");
  frame_l(4500,1,0);if(dut.vl) $fatal(1,"Overrange accepted");
  frame_l(1200,1,0);if(!dut.vl || dut.dl!=1200) $fatal(1,"Recovery");
  // Bad stop bit must invalidate and then recover.
  lrx=0;tick(110);lrx=1;tick(50);if(dut.vl || !dut.el) $fatal(1,"Framing error was not invalidated");
  tick(21000);if(dut.vl) $fatal(1,"Framing error extended freshness");
  frame_l(1500,1,0);if(!dut.vl) $fatal(1,"Framing recovery");
  tick(21000);if(dut.vl || dut.vf || dut.vr) $fatal(1,"Stale timeout");
  // Partial frame gap > 50 ms resets parser.
  byte_l('h5a);byte_l('h5a);byte_l('h15);tick(51000);
  frame_l(1600,1,0);if(!dut.vl || dut.dl!=1600) $fatal(1,"Partial frame gap recovery");
  tick(300000);
  if(command_count!=9 || lines<1 || debug_error) $fatal(1,"Config/telemetry incomplete");
  rst=0;tick(5);if(dut.vl || dut.el) $fatal(1,"Reset clear");
  $display("PASS: 3 UARTs, frame sync/checksum/status/range/framing/gap/timeout, LEDs, config bytes, telemetry, inactive drive outputs");
  $finish;
 end
 initial begin #1000000000;$fatal(1,"Watchdog");end
endmodule
