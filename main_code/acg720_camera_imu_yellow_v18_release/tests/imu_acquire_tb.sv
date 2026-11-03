`timescale 1ns/1ps
// Acquisition/session regression uses a register-accurate transaction stub.
// Electrical I2C protocol and bus-clear behaviour are tested separately.
module imu_recovery_i2c_master #(parameter CLK_HZ=1000000,I2C_HZ=25000)(
 input clk,rst_n,start,input [6:0] address,input read_op,
 input [7:0] reg_address,write_data,input [3:0] read_count,
 inout scl,sda,output reg busy,done,output reg [3:0] error,
 output reg [95:0] rx_data,output [1:0] drive_low,
 output reg [7:0] address_echo,output reg [3:0] ack_map,output reg [7:0] failure_detail);
 reg [7:0] registers[0:127];reg [7:0] delay_count;integer i;
 reg inject=0;assign drive_low=0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   busy<=0;done<=0;error<=0;rx_data<=0;delay_count<=0;address_echo<=0;ack_map<=0;failure_detail<=0;
   for(i=0;i<128;i=i+1)registers[i]<=0;
   registers[7'h75]<=8'h47;registers[7'h2d]<=8'h18;
  end else begin
   done<=0;
   if(start)begin
    busy<=1;delay_count<=20;error<=inject ? 6 : 0;address_echo<={address,1'b0};ack_map<=15;failure_detail<=8'h10;
    if(read_op)begin
     if(reg_address==8'h1f)rx_data<=96'h001000202000000100020003;
     else rx_data<={88'd0,registers[reg_address[6:0]]};
    end else registers[reg_address[6:0]]<=write_data;
   end else if(busy)begin
    if(delay_count==0)begin busy<=0;done<=1;end else delay_count<=delay_count-1'b1;
   end
  end
 end
endmodule
module imu_acquire_tb;
 reg clk=0;always #500 clk=~clk;reg rst_n=0,press=0;
 tri1 scl,sda;wire active,stopped,valid,link,configured,fault,retrying;
 wire [7:0] error,id,retries;wire [20:0] diag;wire [31:0] count;wire [15:0] dt;
 imu_session_guard guard(.clk(clk),.rst_n(rst_n),.press(press),.fault_pulse(fault),
  .link_ok(link),.configured(configured),.error_code(error),.who_am_i(id),.diagnostic(diag),
  .active(active),.stopped(stopped),.saved_error(),.saved_id(),.saved_diagnostic());
 icm42688_acquire #(.CLK_HZ(1000000),.BOOT_CYCLES(10)) dut(.clk(clk),.rst_n(rst_n&&active),
  .scl(scl),.sda(sda),.sample_valid(valid),.ax(),.ay(),.az(),.gx(),.gy(),.gz(),
  .dt_us(dt),.sample_count(count),.sample_hz(),.who_am_i(id),.error_code(error),
  .link_ok(link),.configured(configured),.diagnostic(diag),.fault_pulse(fault),
  .drive_low(),.trace_valid(),.trace_data(),.retrying(retrying),.retry_total(retries));
 integer faults=0;reg [31:0] old_count;
 always @(posedge clk)begin
  if(fault)faults=faults+1;
  if(valid && (dt>50000 || error!=0 || retrying))$fatal(1,"invalid sample published");
 end
 initial begin
  repeat(5)@(negedge clk);rst_n=1;
  repeat(5)@(negedge clk);press=1;@(negedge clk);press=0;
  wait(count>=5);old_count=count;
  dut.u_i2c.inject=1;#100000000;
  if(!active || stopped || faults==0)$fatal(1,"recovery was cancelled by session guard");
  dut.u_i2c.inject=0;wait(count>old_count+5 && error==0 && link);
  if(faults==0 || retries==0)$fatal(1,"fault path not exercised");
  // Explicit user stop still disables acquisition.
  @(negedge clk);press=1;@(negedge clk);press=0;repeat(5)@(negedge clk);
  if(active)$fatal(1,"manual stop failed");
  $display("PASS imu_acquire_tb: six retries, fault, automatic reinit/resume, valid dt, user stop; faults=%0d",faults);$finish;
 end
 initial begin #2000000000;$fatal(1,"acquisition timeout state=%d count=%d",dut.state,count);end
endmodule
