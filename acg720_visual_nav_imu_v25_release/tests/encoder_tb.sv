`timescale 1ns/1ps
module encoder_tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,a=0,b=0;
 wire signed [31:0] count;wire valid;wire [15:0] invalid;
 encoder_quad #(.SAMPLE_CLKS(1000)) dut(clk,rst,a,b,count,valid,invalid);
 task state_ab;
 input [1:0] ab;
 begin @(negedge clk);{a,b}=ab;repeat(8)@(negedge clk);end
 endtask
 initial begin
   repeat(5)@(negedge clk);rst=1;
   state_ab(2'b10);state_ab(2'b11);state_ab(2'b01);state_ab(2'b00);
   @(posedge valid);#1;if(count!=4 || invalid!=0)$fatal(1,"forward quadrature count");
   state_ab(2'b01);state_ab(2'b11);state_ab(2'b10);state_ab(2'b00);
   @(posedge valid);#1;if(count!=-4 || invalid!=0)$fatal(1,"reverse quadrature count");
   state_ab(2'b11);state_ab(2'b00);
   @(posedge valid);#1;if(count!=0 || invalid!=2)$fatal(1,"illegal transitions must not count travel");
   $display("PASS: signed forward/reverse AB counting and illegal-transition diagnostic");$finish;
 end
 initial begin #100000;$fatal(1,"encoder test timeout");end
endmodule
