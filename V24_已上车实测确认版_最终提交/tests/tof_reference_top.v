module tof3_test_top #(
 parameter CLK_HZ=50000000,TOF_BAUD=9600,DEBUG_BAUD=115200,
 TIMEOUT_MS=500,INIT_DELAY_MS=1000,NEAR_MM=300)(
 input clk,reset_n,key_forward_n,key_left_n,key_reverse_n,key_right_n,
 input tof_l_rx,tof_f_rx,tof_r_rx,
 output tof_l_tx,tof_f_tx,tof_r_tx,debug_tx,
 output PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,servo_pwm,
 output reg [7:0] led);
 // Existing drive signals are explicitly inactive in this sensor-only test.
 assign {PWMA,AIN1,AIN2,PWMB,BIN1,BIN2,STBY,servo_pwm}=8'b0;
 reg [2:0] reset_pipe;
 always @(posedge clk or negedge reset_n)
  if(!reset_n) reset_pipe<=0;else reset_pipe<={reset_pipe[1:0],1'b1};
 wire rst_n=reset_pipe[2];
 wire [15:0] dl,df,dr;
 wire [7:0] sl,sf,sr;
 wire vl,vf,vr,el,ef,er;
 wire [15:0] nl,nf,nr;
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD),.TIMEOUT_MS(TIMEOUT_MS)) l(clk,rst_n,tof_l_rx,dl,sl,vl,el,nl);
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD),.TIMEOUT_MS(TIMEOUT_MS)) f(clk,rst_n,tof_f_rx,df,sf,vf,ef,nf);
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD),.TIMEOUT_MS(TIMEOUT_MS)) r(clk,rst_n,tof_r_rx,dr,sr,vr,er,nr);
 // After sensor boot, broadcast short mode / 55 ms / continuous output.
 // Commands are volatile: no configuration-save command is sent.
 reg [31:0] boot;
 reg [31:0] command_wait;
 reg [3:0] cmd_index;
 reg [1:0] cmd_phase;
 reg cmd_start;
 wire cmd_busy,cmd_tx;
 function [7:0] command_byte;
  input [3:0] i;
  begin case(i)
   0,3,6:command_byte=8'ha5;
   1:command_byte=8'h51;2:command_byte=8'hf6;
   4:command_byte=8'h64;5:command_byte=8'h09;
   7:command_byte=8'h45;default:command_byte=8'hea;
  endcase end
 endfunction
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(TOF_BAUD)) config_tx(clk,rst_n,cmd_start,command_byte(cmd_index),cmd_tx,cmd_busy);
 assign tof_l_tx=cmd_tx;assign tof_f_tx=cmd_tx;assign tof_r_tx=cmd_tx;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin boot<=0;command_wait<=0;cmd_index<=0;cmd_phase<=0;cmd_start<=0;end
 else begin
  cmd_start<=0;
  if(boot<(CLK_HZ/1000)*INIT_DELAY_MS) boot<=boot+1;
  else if(command_wait!=0) command_wait<=command_wait-1;
  else if(cmd_index<9) case(cmd_phase)
   0: if(!cmd_busy) begin cmd_start<=1;cmd_phase<=1;end
   1: if(cmd_busy) cmd_phase<=2;
   2: if(!cmd_busy) begin
    cmd_index<=cmd_index+1;cmd_phase<=0;
    if(cmd_index==2 || cmd_index==5) command_wait<=(CLK_HZ/1000)*150;
   end
  endcase
 end
 reg [3:0] keys1,keys2;
 reg [31:0] heartbeat;
 wire [15:0] selected=(!keys2[0])?dl:(!keys2[1])?df:dr;
 wire selected_valid=(!keys2[0])?vl:(!keys2[1])?vf:vr;
 wire [15:0] cm=selected/10;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin keys1<=15;keys2<=15;heartbeat<=0;led<=0;end
 else begin
  keys1<={key_right_n,key_reverse_n,key_left_n,key_forward_n};keys2<=keys1;
  if(heartbeat>=CLK_HZ-1) heartbeat<=0;else heartbeat<=heartbeat+1;
  if(keys2[2:0]!=3'b111) led<=selected_valid?((cm>255)?8'hff:cm[7:0]):0;
  else led<={(heartbeat>=CLK_HZ/2), (el|ef|er|!vl|!vf|!vr),
             (vr && dr<NEAR_MM),(vf && df<NEAR_MM),(vl && dl<NEAR_MM),vr,vf,vl};
 end
 // Fixed-width ASCII snapshot every 250 ms; distance and status are hexadecimal.
 reg [15:0] snap_l,snap_f,snap_r;
 reg [7:0] snap_sl,snap_sf,snap_sr;
 reg [2:0] snap_v;
 reg [31:0] report_timer;
 reg report_active,dbg_start;
 reg [5:0] index;
 reg [1:0] dbg_phase;
 wire dbg_busy;
 reg [7:0] dbg_data;
 function [7:0] hexchar;
  input [3:0] n;
  begin hexchar=(n<10)?(8'h30+n):(8'h41+n-10);end
 endfunction
 reg [15:0] value_d;
 reg [7:0] value_s;
 reg value_v;
 reg [5:0] off;
 always @* begin
  if(index<14) begin off=index;value_d=snap_l;value_s=snap_sl;value_v=snap_v[0];end
  else if(index<28) begin off=index-14;value_d=snap_f;value_s=snap_sf;value_v=snap_v[1];end
  else begin off=index-28;value_d=snap_r;value_s=snap_sr;value_v=snap_v[2];end
  case(off)
   0:dbg_data=(index<14)?8'h4c:(index<28)?8'h46:8'h52;
   1:dbg_data="=";
   2:dbg_data=hexchar(value_d[15:12]);3:dbg_data=hexchar(value_d[11:8]);
   4:dbg_data=hexchar(value_d[7:4]);5:dbg_data=hexchar(value_d[3:0]);
   6:dbg_data="/";7:dbg_data="S";8:dbg_data="=";
   9:dbg_data=hexchar(value_s[7:4]);10:dbg_data=hexchar(value_s[3:0]);
   11:dbg_data="/";12:dbg_data=value_v?"1":"0";default:dbg_data=" ";
  endcase
  if(index==42) dbg_data=13;
  if(index==43) dbg_data=10;
 end
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(DEBUG_BAUD)) telemetry(clk,rst_n,dbg_start,dbg_data,debug_tx,dbg_busy);
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  snap_l<=0;snap_f<=0;snap_r<=0;snap_sl<=8'he0;snap_sf<=8'he0;snap_sr<=8'he0;snap_v<=0;
  report_timer<=0;report_active<=0;dbg_start<=0;index<=0;dbg_phase<=0;
 end else begin
  dbg_start<=0;
  if(report_timer<CLK_HZ/4-1) report_timer<=report_timer+1;
  else if(!report_active) begin
   report_timer<=0;report_active<=1;index<=0;dbg_phase<=0;
   snap_l<=dl;snap_f<=df;snap_r<=dr;snap_sl<=sl;snap_sf<=sf;snap_sr<=sr;snap_v<={vr,vf,vl};
  end
  if(report_active) case(dbg_phase)
   0: if(!dbg_busy) begin dbg_start<=1;dbg_phase<=1;end
   1: if(dbg_busy) dbg_phase<=2;
   2: if(!dbg_busy) begin
    if(index==43) report_active<=0;
    else begin index<=index+1;dbg_phase<=0;end
   end
  endcase
 end
endmodule
