`timescale 1ns/1ps
// Two line mailboxes. Each payload has frame/row identifiers; host publishes
// only complete frames. A busy bank drops a whole line, never overwrites it.
module video_udp_mirror #(parameter WIDTH=640,HEIGHT=480)(
 input pclk,prst_n,input [23:0] rgb,input de,vs,
 input nclk,nrst_n,input tx_done,payload_req,
 output reg tx_start,output [7:0] payload,
 output [15:0] payload_length,output reg dropped_line
);
 localparam LEN=16+WIDTH*2;
 reg [15:0] memory[0:WIDTH*2-1];
 reg [31:0] frame_meta[0:1];reg [15:0] row_meta[0:1];
 reg [1:0] ready_toggle,ack_toggle;
 (* syn_preserve=1 *) reg [1:0] ack_m,ack_s,ready_m,ready_s;
 reg [31:0] frame_id;reg [15:0] row;reg [10:0] x;
 reg de_d,vs_d,bank,line_accept;
 wire free_bank=ready_toggle[bank]==ack_s[bank];
 wire pixel_accept=de && ((!de_d && free_bank) || (de_d && line_accept)) && x<WIDTH;
 wire [10:0] write_x=de_d?x:0;
 wire [15:0] rgb565={rgb[23:19],rgb[15:10],rgb[7:3]};
 always @(posedge pclk)if(prst_n && pixel_accept)
   memory[(bank?WIDTH:0)+write_x]<=rgb565;
 always @(posedge pclk or negedge prst_n)begin
   if(!prst_n)begin ready_toggle<=0;ack_m<=0;ack_s<=0;frame_id<=0;row<=0;x<=0;de_d<=0;vs_d<=0;bank<=0;line_accept<=0;dropped_line<=0;frame_meta[0]<=0;frame_meta[1]<=0;row_meta[0]<=0;row_meta[1]<=0;end
   else begin
     ack_m<=ack_toggle;ack_s<=ack_m;de_d<=de;vs_d<=vs;
     if(vs && !vs_d)begin frame_id<=frame_id+1'b1;row<=0;end
     if(de)begin
       if(!de_d)begin x<=1;line_accept<=free_bank;if(!free_bank)dropped_line<=1;end
       else x<=x+1'b1;
     end else begin
       x<=0;
       if(de_d)begin
         if(line_accept && x==WIDTH && row<HEIGHT)begin
           frame_meta[bank]<=frame_id;row_meta[bank]<=row;
           ready_toggle[bank]<=~ready_toggle[bank];
         end
         row<=row+1'b1;bank<=~bank;line_accept<=0;
       end
     end
   end
 end
 reg active,selected;
 reg [4:0] cooldown;
 reg [15:0] index;
 reg [31:0] tx_frame;reg [15:0] tx_row;
 reg [15:0] word_data;
 wire [15:0] pixel_index=(index<16)?0:((index-16)>>1);
 // Prefetch the next word on the low-byte cycle, current on high-byte cycle.
 wire [15:0] prefetch=(payload_req && index>=16 && index[0] && pixel_index<WIDTH-1)?pixel_index+1'b1:pixel_index;
 always @(posedge nclk)word_data<=memory[(selected?WIDTH:0)+prefetch];
 assign payload_length=LEN;
 reg [7:0] header;
 always @* begin
   case(index)
     0:header="F";1:header="P";2:header="G";3:header="1";
     4:header=tx_frame[31:24];5:header=tx_frame[23:16];6:header=tx_frame[15:8];7:header=tx_frame[7:0];
     8:header=tx_row[15:8];9:header=tx_row[7:0];
     10:header=(WIDTH>>8);11:header=WIDTH;
     12:header=(HEIGHT>>8);13:header=HEIGHT;
     14:header=0;15:header=1;default:header=0;
   endcase
 end
 assign payload=(index<16)?header:(index[0]?word_data[7:0]:word_data[15:8]);
 always @(posedge nclk or negedge nrst_n)begin
   if(!nrst_n)begin ack_toggle<=0;ready_m<=0;ready_s<=0;active<=0;selected<=0;index<=0;tx_frame<=0;tx_row<=0;tx_start<=0;cooldown<=0;end
   else begin
     ready_m<=ready_toggle;ready_s<=ready_m;tx_start<=0;
     if(cooldown!=0)cooldown<=cooldown-1'b1;
     if(!active && cooldown==0)begin
       index<=0;
       if(ready_s[0]!=ack_toggle[0])begin
         active<=1;selected<=0;tx_frame<=frame_meta[0];tx_row<=row_meta[0];tx_start<=1;
       end else if(ready_s[1]!=ack_toggle[1])begin
         active<=1;selected<=1;tx_frame<=frame_meta[1];tx_row<=row_meta[1];tx_start<=1;
       end
     end else begin
       if(payload_req && index<LEN-1)index<=index+1'b1;
       if(tx_done)begin active<=0;ack_toggle[selected]<=ready_s[selected];cooldown<=20;end
     end
   end
 end
endmodule
