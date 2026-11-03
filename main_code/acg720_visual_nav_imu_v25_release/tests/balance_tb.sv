`timescale 1ns/1ps
module balance_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst_n=0,start=0,stop=0,v=0;
 reg signed [31:0] l=0,r=0;
 wire enable,done,fault,ready,busy;wire signed [31:0] target;
 wire [3:0] phase,code;wire [1:0] steering;
 auto_sequence #(.CLK_HZ(10000),.WAIT_MS(5),.LEFT_TRIM_PERCENT(150),.RIGHT_TRIM_PERCENT(100)) dut(
 .clk(clk),.rst_n(rst_n),.start(start),.stop(stop),.count_l(l),.count_r(r),
 .speed_l(phase==4 ? -60:60),.speed_r(phase==4 ? -60:60),
 .valid_l(v),.valid_r(v),.speed_limit_rpm(32'd60),.motor_enable(enable),.target_rpm(target),.steering(steering),
 .phase(phase),.fault_code(code),.busy(busy),.done(done),.fault(fault),.ready(ready));
 integer n=0,model=0,peak_raw=0,diff=0,ms_at_bad=0;
 always @(negedge clk)begin
   v=0;
   if(enable)begin
     if(n==99)begin
       n=0;v=1;
       if(model==0)begin l=phase==4 ? -150:150;r=phase==4 ? -100:100;end
       else begin l=50;r=0;end
     end else n=n+1;
   end else n=0;
 end
 always @(posedge clk)begin
   diff=dut.progress_l-dut.progress_r;
   if(model==0 && diff>peak_raw)peak_raw=diff;
   if(model==1 && dut.imbalance_ms==1 && ms_at_bad==0)ms_at_bad=dut.elapsed;
 end
 task reset_test;
 begin rst_n=0;start=0;repeat(5)@(negedge clk);rst_n=1;repeat(5)@(negedge clk);
 start=1;@(negedge clk);start=0;end
 endtask
 initial begin
   reset_test;wait(done || fault);#1;
   if(!done || fault || peak_raw<=dut.GOAL/4)$fatal(1,"ratio-normalized protection rejected intended trim");
   model=1;reset_test;wait(fault);#1;
   if(code!=9 || enable || dut.fault_phase!=2)$fatal(1,"real mismatch protection missing");
   if(ms_at_bad<500 || dut.elapsed-ms_at_bad<149)$fatal(1,"grace/persistence not applied");
   if(dut.fault_progress_left<=dut.fault_progress_right || dut.progress_l==0)
      $fatal(1,"fault erased left/right evidence");
   repeat(100)@(negedge clk);
   if(dut.progress_l!=dut.fault_progress_left)$fatal(1,"fault evidence changed after stop");
   $display("PASS: intended ratio survives >25cm raw difference; true mismatch persists before fault9; fault counters preserved");
   $finish;
 end
 initial begin #10000000;$fatal(1,"balance test timeout");end
endmodule
