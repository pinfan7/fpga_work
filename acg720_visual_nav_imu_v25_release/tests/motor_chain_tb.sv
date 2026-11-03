`timescale 1ns/1ps
module motor_chain_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,enable=0;
 reg signed [31:0] target=60;
 wire la,la1,la2,rb,rb1,rb2,stby;
 wire signed [31:0] cnt_l,cnt_r,rpm_l,rpm_r,tl,tr;
 wire signed [15:0] pi_l,pi_r;wire [15:0] duty_l,duty_r;
 wire vl,vr;
 car_closed_loop #(.CLK_FREQ(1000000),.PWM_HZ(1000)) dut(
 .clk(clk),.rst_n(rst),.drive_enable(enable),.target_rpm(target),
 .count_left(cnt_l),.count_right(cnt_r),.speed_left(rpm_l),.speed_right(rpm_r),
 .rpm_valid_left(vl),.rpm_valid_right(vr),.debug_target_left(tl),.debug_target_right(tr),
 .debug_pi_left(pi_l),.debug_pi_right(pi_r),.debug_duty_left(duty_l),.debug_duty_right(duty_r),
 .debug_invalid_left(),.debug_invalid_right(),.debug_control_valid(),
 .PWMA(la),.AIN1(la1),.AIN2(la2),.PWMB(rb),.BIN1(rb1),.BIN2(rb2),.STBY(stby),
 .ENC_LA(1'b0),.ENC_LB(1'b0),.ENC_RA(1'b0),.ENC_RB(1'b0),.servo_pwm());
 integer updates=0;
 always @(posedge clk)if(dut.pi_valid_left)updates=updates+1;
 task windows;
 input integer n;
 integer k;
 begin for(k=0;k<n;k=k+1)begin @(posedge vl);repeat(200)@(negedge clk);end end
 endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;enable=1;windows(12);
   if(updates!=12 || pi_l<=0 || pi_r<=0 || duty_l==0 || duty_r==0)
     $fatal(1,"left/right PI/PWM chain not updating from aligned targets");
   if(tl!=63 || tr!=60 || !la1 || la2 || !rb1 || rb2)$fatal(1,"forward targets/direction");
   enable=0;repeat(3)@(negedge clk);
   if(stby || la || rb)$fatal(1,"drive disable did not stop outputs");
   target=-60;enable=1;windows(12);
   if(tl!=-63 || tr!=-60 || pi_l>=0 || pi_r>=0 || duty_l==0 || duty_r==0)
     $fatal(1,"reverse PI/PWM chain");
   if(la1 || !la2 || rb1 || !rb2)$fatal(1,"reverse direction");
   $display("PASS: both real encoder-window/RPM/target/PI/PWM pipelines update; forward/reverse polarity; stop gate");
   $finish;
 end
 initial begin #10000000;$fatal(1,"motor chain timeout");end
endmodule
