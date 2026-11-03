`timescale 1ns/1ps
// GY-53L1 UART modules, not bare VL53L1X I2C sensors. RX lines stay separate.
module tof3_subsystem #(
 parameter integer CLK_HZ=50000000,BAUD=9600,TIMEOUT_MS=500,
 parameter integer INIT_DELAY_MS=1000,COMMAND_WAIT_MS=150
)(
 input clk,rst_n,tof_l_rx,tof_f_rx,tof_r_rx,
 output tof_l_tx,tof_f_tx,tof_r_tx,configured,
 output [15:0] distance_l,distance_f,distance_r,
 output [7:0] status_l,status_f,status_r,
 output [2:0] valid,fault_seen,
 output [15:0] frames_l,frames_f,frames_r
);
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.TIMEOUT_MS(TIMEOUT_MS)) l(
   clk,rst_n,tof_l_rx,distance_l,status_l,valid[0],fault_seen[0],frames_l);
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.TIMEOUT_MS(TIMEOUT_MS)) f(
   clk,rst_n,tof_f_rx,distance_f,status_f,valid[1],fault_seen[1],frames_f);
 tof_channel #(.CLK_HZ(CLK_HZ),.BAUD(BAUD),.TIMEOUT_MS(TIMEOUT_MS)) r(
   clk,rst_n,tof_r_rx,distance_r,status_r,valid[2],fault_seen[2],frames_r);
 // Same volatile setup sequence as the successful three-sensor test.
 reg [31:0] boot,command_wait;
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
 uart_tx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) config_tx(
   clk,rst_n,cmd_start,command_byte(cmd_index),cmd_tx,cmd_busy);
 assign tof_l_tx=cmd_tx;
 assign tof_f_tx=cmd_tx;
 assign tof_r_tx=cmd_tx;
 // Configuration bytes sent; sensor response is still required separately.
 assign configured=cmd_index==9;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin boot<=0;command_wait<=0;cmd_index<=0;cmd_phase<=0;cmd_start<=0;end
 else begin
   cmd_start<=0;
   if(boot<(CLK_HZ/1000)*INIT_DELAY_MS)boot<=boot+1'b1;
   else if(command_wait!=0)command_wait<=command_wait-1'b1;
   else if(cmd_index<9)case(cmd_phase)
     0:if(!cmd_busy)begin cmd_start<=1;cmd_phase<=1;end
     1:if(cmd_busy)cmd_phase<=2;
     2:if(!cmd_busy)begin
       cmd_index<=cmd_index+1'b1;cmd_phase<=0;
       if(cmd_index==2 || cmd_index==5)command_wait<=(CLK_HZ/1000)*COMMAND_WAIT_MS;
     end
     default:cmd_phase<=0;
   endcase
 end
endmodule
