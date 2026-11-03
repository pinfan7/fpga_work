module uart_rx #(parameter CLK_HZ=50000000, BAUD=9600)(
 input clk, rst_n, rx, output reg [7:0] data,
 output reg ready, output reg error);
 localparam N=(CLK_HZ+BAUD/2)/BAUD;
 reg [1:0] sync;
 reg [2:0] state;
 reg [31:0] count;
 reg [2:0] bitno;
 reg [7:0] shift;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin sync<=3;state<=0;count<=0;bitno<=0;shift<=0;data<=0;ready<=0;error<=0;end
 else begin
  sync<={sync[0],rx}; ready<=0;error<=0;
  case(state)
   0: if(!sync[1]) begin state<=1;count<=N/2-1;end
   1: if(count!=0) count<=count-1;
      else if(!sync[1]) begin state<=2;count<=N-1;bitno<=0;end
      else state<=0;
   2: if(count!=0) count<=count-1;
      else begin shift[bitno]<=sync[1];count<=N-1;
       if(bitno==7) state<=3;else bitno<=bitno+1;end
   3: if(count!=0) count<=count-1;
      else begin
       if(sync[1]) begin state<=0;data<=shift;ready<=1;end
       else begin error<=1;state<=4;end
      end
   4: if(sync[1]) state<=0;
  endcase
 end
endmodule
