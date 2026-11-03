`timescale 1ns/1ps
module top_route_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,s1=0,s3=1;
 reg [15:0] dl=4000,df=4000,dr=4000,frames=0;
 reg [2:0] valid=7;
 wire stby,servo;wire [7:0] leds;
 ackermann_auto_top #(.CLK_HZ(1000000),.START_WAIT_MS(60)) dut(
   .clk(clk),.reset_n(rst),.key_forward_n(s1),.key_reverse_n(s3),
   .source_key_n(1'b1),.imu_recal_n(1'b1),
   .ENC_LA(1'b0),.ENC_LB(1'b0),.ENC_RA(1'b0),.ENC_RB(1'b0),
   .tof_l_rx(1'b1),.tof_f_rx(1'b1),.tof_r_rx(1'b1),
   .STBY(stby),.servo_pwm(servo),.led(leds));
 integer cycles=0,width=0;
 always @(negedge clk)begin cycles=cycles+1;if(cycles%10000==0)frames=frames+1;end
 task ticks;
   input integer n;begin repeat(n)@(negedge clk);end
 endtask
 task measure;
   input integer expected;
   begin
     @(negedge servo);@(posedge servo);width=0;
     while(servo)begin @(negedge clk);if(servo)width=width+1;end
     if(width<expected-1 || width>expected+1)$fatal(1,"actual pad pulse expected %0d got %0d",expected,width);
   end
 endtask
 initial begin
   force dut.tof_configured=1'b1;
   force dut.tof_valid=valid;
   force dut.tof_dl=dl;force dut.tof_df=df;force dut.tof_dr=dr;
   force dut.tof_nl=frames;force dut.tof_nf=frames;force dut.tof_nr=frames;
   ticks(10);rst=1;ticks(600000);
   if(stby || servo || dut.armed)$fatal(1,"held start enabled servo or motor");
   s1=1;ticks(30000);s1=0;ticks(30000);s1=1;
   if(dut.busy || servo || stby)$fatal(1,"started without calibrated IMU");
   dut.u_vision.nav_imu_ok=1;ticks(30000);
   s1=0;ticks(25000);s1=1;
   wait(dut.phase==1);measure(1400);
   wait(stby);df=300;
   wait(dut.phase==3);measure(1039);
   // Actual production pad gate must react without waiting for the state machine.
   dut.u_vision.nav_imu_ok=0;@(posedge clk);#1;
   if(stby)$fatal(1,"IMU invalid did not gate physical motor output");
   ticks(25000);if(!dut.fault || dut.code!=8 || servo)$fatal(1,"IMU fault did not latch or stop servo session");
   // New session with the left side less free selects the right path.
   rst=0;s1=1;ticks(10);rst=1;dut.u_vision.nav_imu_ok=1;
   dl=800;df=300;dr=4000;ticks(550000);
   s1=0;ticks(25000);s1=1;wait(dut.phase==3);measure(1850);
   s3=0;#1;if(stby)$fatal(1,"S3 pad inhibit delayed");
   ticks(25000);if(servo || dut.busy)$fatal(1,"S3 did not terminate servo session");
   $display("PASS: real S1 held-key gate; IMU readiness; actual 1400/1039/1850us pad pulses; raw sensor gate; fault latch; S3 stop; real motor/servo modules with vision boundary model");
   $finish;
 end
 initial begin #50000000;$fatal(1,"top route timeout phase=%0d fault=%0d",dut.phase,dut.code);end
endmodule
