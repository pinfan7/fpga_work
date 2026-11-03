module tof_channel #(parameter CLK_HZ=50000000,BAUD=9600,TIMEOUT_MS=500)(
 input clk,rst_n,rx,output reg [15:0] distance_mm,
 output reg [7:0] status_byte, output reg valid,output reg fault_seen,
 output reg [15:0] good_frames);
 localparam LIMIT=(CLK_HZ/1000)*TIMEOUT_MS;
 localparam GAP=CLK_HZ/20;
 wire [7:0] data;
 wire ready,rx_error;
 uart_rx #(.CLK_HZ(CLK_HZ),.BAUD(BAUD)) u_rx(clk,rst_n,rx,data,ready,rx_error);
 reg [2:0] pos;
 reg [7:0] sum,hi,lo,mode;
 reg [31:0] age,gap_age;
 always @(posedge clk or negedge rst_n)
 if(!rst_n) begin
  pos<=0;sum<=0;hi<=0;lo<=0;mode<=0;distance_mm<=0;status_byte<=8'he0;
  valid<=0;fault_seen<=0;good_frames<=0;age<=LIMIT;gap_age<=0;
 end else begin
  if(age<LIMIT) age<=age+1;else valid<=0;
  if(pos!=0) begin
   if(gap_age>=GAP) begin pos<=0;gap_age<=0;fault_seen<=1;valid<=0;end
   else gap_age<=gap_age+1;
  end
  if(rx_error) begin pos<=0;fault_seen<=1;valid<=0;end
  else if(ready) begin
   gap_age<=0;
   case(pos)
    0: if(data==8'h5a) begin pos<=1;sum<=8'h5a;end
    1: if(data==8'h5a) begin pos<=2;sum<=8'hb4;end else pos<=0;
    2: if(data==8'h15) begin pos<=3;sum<=sum+data;end
       else if(data==8'h5a) begin pos<=2;sum<=8'hb4;end else pos<=0;
    3: if(data==3) begin pos<=4;sum<=sum+data;end
       else begin pos<=(data==8'h5a)?1:0;sum<=8'h5a;fault_seen<=1;valid<=0;end
    4: begin hi<=data;sum<=sum+data;pos<=5;end
    5: begin lo<=data;sum<=sum+data;pos<=6;end
    6: begin mode<=data;sum<=sum+data;pos<=7;end
    7: begin
     if(data==sum) begin
      status_byte <= mode;
        if(mode[7:4]==0 && {hi,lo}>=50 && {hi,lo}<=4000) begin
         distance_mm <= {hi,lo};
            age <= 0;
            valid <= 1;
            end
      if(mode[7:4]!=0 || {hi,lo}<50 || {hi,lo}>4000) begin fault_seen<=1;valid<=0;end
      if(good_frames!=16'hffff) good_frames<=good_frames+1;
      pos<=0;
     end else begin 
        fault_seen<=1;valid<=0;pos<=(data==8'h5a)?1:0;sum<=8'h5a;end
    end
   endcase
  end
 end
endmodule
