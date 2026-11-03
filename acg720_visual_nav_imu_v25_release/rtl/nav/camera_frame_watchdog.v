// Input toggle changes only when DDR commits a NEW complete camera frame.
// Repeated display of the last frame must not keep autonomous motion alive.
module camera_frame_watchdog #(
 parameter integer CLK_HZ=50000000,TIMEOUT_MS=300
)(input clk,rst_n,commit_toggle,source_enabled,output live);
 (* syn_preserve=1 *) reg cm,cs,em,es;
 reg seen,received;reg [31:0] divider,age;
 wire tick=divider==CLK_HZ/1000-1;
 assign live=es && received && age<TIMEOUT_MS;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin cm<=0;cs<=0;em<=0;es<=0;seen<=0;received<=0;divider<=0;age<=TIMEOUT_MS;end
 else begin
   cm<=commit_toggle;cs<=cm;em<=source_enabled;es<=em;
   divider<=tick?0:divider+1'b1;
   if(!es)begin received<=0;age<=TIMEOUT_MS;seen<=cs;end
   else if(cs!=seen)begin seen<=cs;age<=0;received<=1;end
   else if(tick && age<TIMEOUT_MS)age<=age+1'b1;
 end
endmodule
