module uart_tx #(parameter CLK_HZ=50000000, BAUD=9600)(
 input clk,rst_n,start,input [7:0] data, output reg tx,output reg busy);
 localparam N=(CLK_HZ+BAUD/2)/BAUD;
 reg [9:0] bits;
 reg [3:0] bitno;
 reg [31:0] count;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin tx<=1;busy<=0;bits<=10'h3ff;bitno<=0;count<=0;end
 else if(!busy) begin
  tx<=1;
  if(start) begin bits<={1'b1,data,1'b0};tx<=0;busy<=1;bitno<=0;count<=N-1;end
 end else if(count!=0) count<=count-1;
 else begin
  count<=N-1;
  if(bitno==9) begin busy<=0;tx<=1;end
  else begin bitno<=bitno+1;bits<={1'b1,bits[9:1]};tx<=bits[1];end
 end
endmodule
