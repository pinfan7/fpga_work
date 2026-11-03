`timescale 1ns/1ps
// Real three UART receivers + guard + motion sequencer + motor/servo logic.
// Vendor video replaced at boundary. Encoder feedback injected at 100Hz here;
// separate motor_chain and encoder tests exercise actual quadrature decoding.
module tof_drive_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,s1=0,s3=1;
 reg [2:0] rx=3'b111,feed=0;
 reg [15:0] dl=1000,df=1000,dr=1000;
 reg [7:0] sl=1,sf=1,sr=1;
 reg [2:0] corrupt=0;
 wire [2:0] tx;
 wire stby,pa,pb,a1,a2,b1,b2,servo;
 wire [7:0] led;
 ackermann_auto_top #(.CLK_HZ(1000000),.TOF_BAUD(100000),
   .TOF_INIT_DELAY_MS(2),.TOF_COMMAND_WAIT_MS(3),
   .TOF_TIMEOUT_MS(120),.TOF_CLEAR_MS(30),.TOF_BRAKE_MS(30)) dut(
   .clk(clk),.reset_n(rst),.key_forward_n(s1),.key_reverse_n(s3),
   .source_key_n(1'b1),.imu_recal_n(1'b1),
   .ENC_LA(1'b0),.ENC_LB(1'b0),.ENC_RA(1'b0),.ENC_RB(1'b0),
   .tof_l_rx(rx[0]),.tof_f_rx(rx[1]),.tof_r_rx(rx[2]),
   .tof_l_tx(tx[0]),.tof_f_tx(tx[1]),.tof_r_tx(tx[2]),
   .STBY(stby),.PWMA(pa),.PWMB(pb),.AIN1(a1),.AIN2(a2),.BIN1(b1),.BIN2(b2),
   .servo_pwm(servo),.led(led));
 task automatic tick(input integer n);repeat(n)@(negedge clk);endtask
 task automatic send_byte(input integer ch,input [7:0] b);
 integer i;begin
   rx[ch]=0;tick(10);
   for(i=0;i<8;i=i+1)begin rx[ch]=b[i];tick(10);end
   rx[ch]=1;tick(10);
 end endtask
 task automatic frame(input integer ch,input [15:0] d,input [7:0] mode,input bad);
 reg [7:0] sum;begin
   sum=8'hcc+d[15:8]+d[7:0]+mode;
   send_byte(ch,8'h5a);send_byte(ch,8'h5a);send_byte(ch,8'h15);send_byte(ch,3);
   send_byte(ch,d[15:8]);send_byte(ch,d[7:0]);send_byte(ch,mode);send_byte(ch,sum^bad);
 end endtask
 initial forever begin wait(rst && feed[0]);frame(0,dl,sl,corrupt[0]);tick(10000);end
 initial forever begin wait(rst && feed[1]);frame(1,df,sf,corrupt[1]);tick(10000);end
 initial forever begin wait(rst && feed[2]);frame(2,dr,sr,corrupt[2]);tick(10000);end
 wire [7:0] cmd;wire cmd_valid,cmd_error;
 integer commands=0;reg [7:0] expected[0:8];
 uart_rx #(.CLK_HZ(1000000),.BAUD(100000)) monitor(clk,rst,tx[0],cmd,cmd_valid,cmd_error);
 always @(posedge clk)if(rst)begin
   if(tx[0]!==tx[1] || tx[0]!==tx[2])$fatal(1,"broadcast config differs");
   if(cmd_error)$fatal(1,"config UART framing");
   if(cmd_valid)begin
     if(commands>=9 || cmd!==expected[commands])$fatal(1,"config byte wrong");
     commands=commands+1;
   end
 end
 reg feedback=0;reg model_valid=0;reg signed [31:0] model_count=0;
 integer count_div=0;
 always @(negedge clk)begin
   model_valid=0;
   if(count_div==9999)begin
     count_div=0;model_valid=dut.motor_run;model_count=feedback ? 80:0;
   end else count_div=count_div+1;
 end
 task press_start;
 begin s1=0;tick(30000);s1=1;tick(30000);end
 endtask
 task recover;
 begin dl=1000;df=1000;dr=1000;sl=1;sf=1;sr=1;corrupt=0;feed=7;
   tick(180000);if(!dut.tof_can_start)$fatal(1,"clear did not qualify valid=%b distance=%d/%d/%d",dut.tof_valid,dut.tof_dl,dut.tof_df,dut.tof_dr);end
 endtask
 task start_moving;
 begin press_start;wait(dut.phase==2);tick(10000);
   if(!stby || dut.tof_latched || !dut.servo_session)$fatal(1,"start did not enable movement/servo");end
 endtask
 initial begin
   expected[0]='ha5;expected[1]='h51;expected[2]='hf6;
   expected[3]='ha5;expected[4]='h64;expected[5]='h09;
   expected[6]='ha5;expected[7]='h45;expected[8]='hea;
   force dut.count_l=model_count;force dut.count_r=model_count;
   force dut.valid_l=model_valid;force dut.valid_r=model_valid;
   tick(5);rst=1;tick(600000);
   if(dut.armed || stby || servo || dut.busy)$fatal(1,"held S1 starts on boot");
   if(commands!=9 || !dut.tof_configured)$fatal(1,"setup incomplete");
   s1=1;tick(30000);press_start;
   if(dut.busy || servo || stby)$fatal(1,"started without TOF data");
   recover;
   if(!led[5])$fatal(1,"ready LED missing");
   start_moving;tick(500000);
   if(dut.target<=20)$fatal(1,"did not accelerate outside slow zone");
   dl=600;tick(20000);
   if(!dut.tof_slow || dut.target>20 || !stby)$fatal(1,"left slow limit failed");
   dl=720;tick(20000);
   if(!dut.tof_slow)$fatal(1,"slow hysteresis failed");
   dl=800;tick(20000);
   if(dut.tof_slow)$fatal(1,"slow clear failed");
   // Camera recognition continues while the controller is running.
   repeat(5)dut.u_vision.send_frame();
   if(!dut.u_vision.box_valid || !stby)$fatal(1,"concurrent vision failed");
   df=300;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=2 || !stby || pa || pb || {a1,a2,b1,b2}!=4'b1111)
     $fatal(1,"front hazard / short brake wrong");
   tick(40000);
   if(stby || dut.phase!=12 || !led[3])$fatal(1,"stop did not latch / brake expire");
   recover;tick(100000);
   if(stby || dut.busy || !dut.tof_latched)$fatal(1,"automatically resumed");
   // Valid recovery requires a fresh S1 edge. S3 outranks active braking.
   start_moving;dr=300;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=4)$fatal(1,"right channel veto missing");
   s3=0;#1;
   if(stby || pa || pb || a1 || a2 || b1 || b2)$fatal(1,"S3 did not override brake");
   tick(40000);s3=1;tick(30000);recover;
   start_moving;dl=300;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=1)$fatal(1,"left channel veto missing");
   recover;start_moving;sl=8'h21;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=8 || dut.tof_valid[0])$fatal(1,"invalid status kept old range");
   recover;start_moving;corrupt[2]=1;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=8 || dut.tof_valid[2])$fatal(1,"checksum did not veto");
   recover;start_moving;feed[1]=0;wait(dut.tof_latched);tick(10);
   if(dut.tof_reason!=8 || dut.tof_valid[1])$fatal(1,"stale channel did not stop");
   // Stopped car still processes camera frames and IMU session.
   repeat(5)dut.u_vision.send_frame();
   if(!dut.u_vision.box_valid || !dut.u_vision.active)$fatal(1,"TOF stop stopped video/IMU");
   recover;start_moving;
   // Video faults do not veto drive; TOF still does.
   dut.u_vision.freeze_pixels=1;dut.u_vision.video_fault=1;dut.u_vision.imu_fault=1;
   tick(30000);if(!stby || dut.fault)$fatal(1,"vision fault affects motion");
   dut.u_vision.freeze_pixels=0;dut.u_vision.video_fault=0;dut.u_vision.imu_fault=0;
   feedback=1;wait(dut.done || dut.fault);tick(10);
   if(!dut.done || dut.fault || stby || dut.tof_latched)$fatal(1,"forward 1m completion failed");
   if(dut.steering!=0)$fatal(1,"forward-only changed steering");
   tick(200000);if(dut.busy || stby)$fatal(1,"reverse/turn or auto restart after done");
   rst=0;tick(5);
   if(stby || pa || pb || servo)$fatal(1,"reset outputs unsafe");
   $display("PASS: actual 3 UART config/frames; held-key boot; missing sensor inhibit; slow/hysteresis; left/front/right stop; bounded brake; S3 priority; bad status/checksum/stale veto; manual-only restart with servo; forward distance only; independent camera+IMU");
   $finish;
 end
 initial begin #250000000;$fatal(1,"integration watchdog phase=%d reason=%d",dut.phase,dut.tof_reason);end
endmodule
