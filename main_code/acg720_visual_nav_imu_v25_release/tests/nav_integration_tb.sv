`timescale 1ns/1ps
// Actual UART parsing, key debounce, navigator, motor/servo and output safety.
// Vendor video IP is modeled; CDC/detector have separate verification.
module nav_integration_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,s1=0,s3=1;
 reg [2:0] rx=7;
 reg [15:0] df=1000;
 wire stby,pa,pb,a1,a2,b1,b2,servo;wire [7:0] led;
 ackermann_auto_top #(.CLK_HZ(1000000),.TOF_BAUD(100000),.NAV_WAIT_MS(30),
   .TOF_INIT_DELAY_MS(2),.TOF_COMMAND_WAIT_MS(3),.TOF_TIMEOUT_MS(120)) dut(
   .clk(clk),.reset_n(rst),.key_forward_n(s1),.key_reverse_n(s3),
   .source_key_n(1'b1),.imu_recal_n(1'b1),
   .ENC_LA(1'b0),.ENC_LB(1'b0),.ENC_RA(1'b0),.ENC_RB(1'b0),
   .tof_l_rx(rx[0]),.tof_f_rx(rx[1]),.tof_r_rx(rx[2]),
   .STBY(stby),.PWMA(pa),.PWMB(pb),.AIN1(a1),.AIN2(a2),.BIN1(b1),.BIN2(b2),
   .servo_pwm(servo),.led(led));
 // Preserve 1000 clocks/PWM period while accelerating the fabric to 1MHz.
 defparam dut.motors.PWM_HZ=1000;
 task tick(input integer n);repeat(n)@(negedge clk);endtask
 task automatic send_byte(input integer ch,input [7:0] b);
 integer i;begin
   rx[ch]=0;tick(10);
   for(i=0;i<8;i=i+1)begin rx[ch]=b[i];tick(10);end
   rx[ch]=1;tick(10);
 end endtask
 task automatic frame(input integer ch,input [15:0] d);
 reg [7:0] sum;begin
   sum=8'hcd+d[15:8]+d[7:0];
   send_byte(ch,'h5a);send_byte(ch,'h5a);send_byte(ch,'h15);send_byte(ch,3);
   send_byte(ch,d[15:8]);send_byte(ch,d[7:0]);send_byte(ch,1);send_byte(ch,sum);
 end endtask
 initial forever begin wait(rst);frame(0,1000);tick(20000);end
 initial forever begin wait(rst);frame(1,df);tick(20000);end
 initial forever begin wait(rst);frame(2,1000);tick(20000);end
 task press_start;begin s1=0;tick(30000);s1=1;tick(50000);end endtask
 integer pwm_count=0,servo_count=0;
 always @(posedge clk)begin if(pa)pwm_count=pwm_count+1;if(servo)servo_count=servo_count+1;end
 initial begin
   // Actual encoder/RPM/PI pipeline runs; stationary AB is sufficient to
   // verify commanded output waveforms before the 3s no-motion watchdog.
   tick(5);rst=1;
   dut.u_vision.target_valid=1;dut.u_vision.target_live=1;
   tick(650000);
   if(dut.armed || stby || servo)$fatal(1,"Held S1 boot start");
   s1=1;tick(30000);press_start();
   if(!dut.motor_run || !stby || !led[0])$fatal(1,"Did not start phase%d code%d",dut.phase,dut.code);
   tick(100000);
   if(pwm_count==0 || servo_count==0)$fatal(1,"No real motor/servo PWM pwm=%d servo=%d session=%b pulse=%d pi=%d duty=%d run=%b",pwm_count,servo_count,dut.servo_session,dut.pulse,dut.motor_pi_l,dut.motor_duty_l,dut.motor_run);
   dut.u_vision.target_x0=220;dut.u_vision.target_x1=260;tick(220000);
   if(dut.nav_pulse>=1400)$fatal(1,"Target did not steer left");
   
   dut.u_vision.target_x0=300;dut.u_vision.target_x1=340;tick(220000);
   df=100;tick(30000);
   if(!dut.done || pa || pb || !stby || {a1,a2,b1,b2}!=4'b1111)$fatal(1,"Arrival short brake wrong");
   tick(310000);if(stby || !led[4])$fatal(1,"Arrival latch/short brake timeout wrong");
   df=1000;tick(80000);if(dut.motor_run)$fatal(1,"Automatic restart");
   press_start();if(!dut.motor_run)$fatal(1,"S1 restart failed");
   s3=0;#1;if(pa || pb || stby)$fatal(1,"Raw S3 failed immediate gating");
   tick(100);if(dut.busy)$fatal(1,"S3 not latched");
   s3=1;tick(30000);press_start();if(!dut.motor_run)$fatal(1,"S3 restart failed");
   dut.u_vision.imu_fault=1;tick(10);
   if(dut.motor_run || !dut.fault || dut.code!=10)$fatal(1,"IMU fault did not inhibit navigation");
   tick(310000);if(stby)$fatal(1,"IMU brake unbounded");
   dut.u_vision.imu_fault=0;
   tick(150000);if(dut.motor_run)$fatal(1,"IMU recovery auto-restarted");
   press_start();if(!dut.motor_run)$fatal(1,"IMU recovery manual restart failed");
   dut.u_vision.freeze_imu=1;tick(105000);
   if(dut.motor_run || !dut.fault || dut.code!=10)$fatal(1,"Stale IMU did not stop navigation");
   dut.u_vision.freeze_imu=0;tick(350000);press_start();
   if(!dut.motor_run)$fatal(1,"Fresh IMU did not permit manual restart");
   dut.u_vision.target_live=0;tick(10);if(pa || pb || !dut.fault)$fatal(1,"Video lost did not stop");
   tick(310000);if(stby)$fatal(1,"Lost-video brake did not expire");
   $display("PASS nav_integration: held-key boot, UART TOF, real PWM/servo, visual steering, IMU fault/stale brake and manual recovery, 100mm stop, restart, immediate S3, stale-source stop");$finish;
 end
 initial begin #60000000;$fatal(1,"Timeout");end
endmodule
