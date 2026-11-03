`timescale 1ns/1ps
module nav_limits_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,start=0;
 reg signed [31:0] x=0,y=0,h=0,cl=1;
 reg [15:0] frames=0;
 wire motor,done,fault;wire [3:0] phase,code;
 nav_controller #(.CLK_HZ(1000),.WAIT_MS(2)) dut(.clk(clk),.rst_n(rst),.start(start),.stop(1'b0),
   .x_q8(x),.y_q8(y),.heading_q16(h),.imu_ok(1'b1),.imu_update(1'b1),.pose_fault(1'b0),
   .count_l(cl),.count_r(32'sd1),.speed_l(32'sd20),.speed_r(32'sd20),.valid_l(1'b1),.valid_r(1'b1),
   .tof_l(16'd4000),.tof_f(16'd4000),.tof_r(16'd4000),.frames_l(frames),.frames_f(frames),.frames_r(frames),.tof_valid(3'b111),
   .motor_enable(motor),.busy(),.ready(),.done(done),.fault(fault),.avoid_mode(),.target_rpm(),.servo_us(),
   .phase(phase),.fault_code(code),.bypass_left(),.elapsed_ms(),.desired_heading_q16(),.pass_x_mm(),.fresh_l(),.fresh_r(),.avoid_count());
 task begin_case;
   begin
     @(negedge clk);rst=0;start=0;x=0;y=0;h=0;cl=1;
     repeat(3)@(negedge clk);rst=1;
     repeat(3)@(negedge clk);start=1;
     @(negedge clk);start=0;wait(phase==2);@(negedge clk);
   end
 endtask
 initial begin
   begin_case();x=2000*256;y=50*256;
   repeat(3)@(negedge clk);if(!fault || code!=13 || done || motor)$fatal(1,"off-route endpoint falsely completed");
   begin_case();x=2000*256;h=5*65536;
   repeat(3)@(negedge clk);if(!fault || code!=13 || done)$fatal(1,"wrong-heading endpoint falsely completed");
   begin_case();x=2000*256;
   repeat(3)@(negedge clk);if(!done || fault || motor)$fatal(1,"aligned endpoint did not finish");
   begin_case();h=80*65536;
   repeat(3)@(negedge clk);if(!fault || code!=9)$fatal(1,"heading divergence did not stop");
   begin_case();cl=-10;
   repeat(3)@(negedge clk);if(!fault || code!=4)$fatal(1,"encoder reverse did not stop");
   $display("PASS: endpoint needs both lateral and heading alignment; X goal success; heading divergence; encoder reverse");$finish;
 end
endmodule
