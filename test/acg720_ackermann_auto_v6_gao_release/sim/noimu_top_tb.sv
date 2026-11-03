`timescale 1ns/1ps
module noimu_top_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,s1=0,s3=1; // Held S1 at boot must not launch.
 tri scl,sda;pullup(scl);pullup(sda);
 wire servo,stby;wire [7:0] led;
 ackermann_auto_top #(.CLK_HZ(1000000)) dut(
 .clk(clk),.reset_n(rst),.key_forward_n(s1),.key_reverse_n(s3),
 .ENC_LA(1'b0),.ENC_LB(1'b0),.ENC_RA(1'b0),.ENC_RB(1'b0),
 .PWMA(),.AIN1(),.AIN2(),.PWMB(),.BIN1(),.BIN2(),.STBY(stby),
 .servo_pwm(servo),.imu_scl(scl),.imu_sda(sda),.led(led));
 integer high_count;
 task pulse_width;
 input integer expected;
 begin
   @(posedge servo);high_count=0;
   while(servo)begin @(negedge clk);if(servo)high_count=high_count+1;end
   if(high_count!=expected)$fatal(1,"servo width %0d expected %0d",high_count,expected);
 end
 endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;
   repeat(600000)@(negedge clk);
   if(dut.armed || dut.busy || servo || stby)$fatal(1,"held start launched");
   if(scl!==1'b1 || sda!==1'b1)$fatal(1,"IMU lines not released");
   // A stuck/unpowered IMU may pull a line low; it must have no control effect.
   force scl=0;force sda=0;
   s1=1;repeat(30000)@(negedge clk);
   if(!dut.armed || !dut.ready || !led[5])$fatal(1,"no-IMU ready failed");
   s1=0;repeat(30000)@(negedge clk);s1=1;
   if(dut.phase!=1 || dut.fault)$fatal(1,"start still depends on IMU");
   pulse_width(1500);
   force dut.steering=1;repeat(40000)@(negedge clk);pulse_width(1333);
   force dut.steering=2;repeat(40000)@(negedge clk);pulse_width(1667);
   release dut.steering;
   // Confirm preserved wheel compensation for both signs.
   force dut.enable=1;
   force dut.target=60;@(posedge dut.motors.rpm_valid_left);#1;
   if(dut.motors.target_left!=63 || dut.motors.target_right!=60)$fatal(1,"forward trim lost");
   force dut.target=-60;@(negedge dut.motors.rpm_valid_left);
   @(posedge dut.motors.rpm_valid_left);#1;
   if(dut.motors.target_left!=-63 || dut.motors.target_right!=-60)$fatal(1,"reverse trim lost");
   release dut.target;
   release dut.enable;
   wait(stby);@(negedge clk);s3=0;#1;
   if(stby)$fatal(1,"S3 did not inhibit motor immediately");
   repeat(40000)@(negedge clk);
   if(servo || dut.busy || dut.fault)$fatal(1,"abort did not finish cleanly");
   $display("PASS: held start, idle IMU, disconnected/stuck IMU independence, 1500/1333/1667us pulses, 5%% trim, S3");
   $finish;
 end
 initial begin #30000000;$fatal(1,"top test timeout");end
endmodule
