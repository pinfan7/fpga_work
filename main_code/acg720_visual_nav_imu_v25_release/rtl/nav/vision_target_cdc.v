// Bundled-data mailbox. Payload stays unchanged until destination acknowledges.
// Invalid frames are messages too; never retain a valid target indefinitely.
module vision_target_cdc(
 input src_clk,dst_clk,rst_n,src_frame,src_valid,
 input [11:0] src_x0,src_y0,src_x1,src_y1,
 output reg update,valid,
 output reg [11:0] x0,y0,x1,y1
);
 reg req,ack,frame_d;
 reg [48:0] payload;
 (* syn_preserve=1 *) reg ack_m,ack_s,req_m,req_s;
 always @(posedge src_clk or negedge rst_n)
 if(!rst_n)begin req<=0;frame_d<=0;payload<=0;ack_m<=0;ack_s<=0;end
 else begin
   ack_m<=ack;ack_s<=ack_m;frame_d<=src_frame;
   if(src_frame && !frame_d && req==ack_s)begin
     payload<={src_valid,src_x0,src_y0,src_x1,src_y1};req<=~req;
   end
 end
 always @(posedge dst_clk or negedge rst_n)
 if(!rst_n)begin req_m<=0;req_s<=0;ack<=0;update<=0;valid<=0;x0<=0;y0<=0;x1<=0;y1<=0;end
 else begin
   req_m<=req;req_s<=req_m;update<=0;
   if(req_s!=ack)begin
     {valid,x0,y0,x1,y1}<=payload;update<=1;ack<=req_s;
   end
 end
endmodule
