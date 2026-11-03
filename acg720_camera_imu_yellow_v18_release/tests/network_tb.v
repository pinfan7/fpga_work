`timescale 1ns/1ps
module network_tb;
 reg pclk=0,nclk=0,rst=0;always #20 pclk=~pclk;always #4 nclk=~nclk;
 reg [23:0] rgb=0;reg de=0,vs=0;
 wire start,done,req,drop,enable;wire[7:0]payload,gmii;wire[15:0]len;
 video_udp_mirror #(.WIDTH(640),.HEIGHT(4)) dut(pclk,rst,rgb,de,vs,nclk,rst,done,req,start,payload,len,drop);
 eth_udp_tx_gmii udp(.clk125m(nclk),.reset_p(!rst),.tx_en_pulse(start),.tx_done(done),
 .dst_mac(48'hffffffffffff),.src_mac(48'h020000000002),.dst_ip(32'hc0a80003),.src_ip(32'hc0a80002),
 .dst_port(16'd6102),.src_port(16'd5000),.data_length(len),.payload_req_o(req),.payload_dat_i(payload),
 .gmii_tx_clk(),.gmii_txen(enable),.gmii_txd(gmii));
 integer f,j,y,x,out,cnt=0,packets=0,gap=100;reg was=0;
 initial out=$fopen("reports/network_packets.hex","w");
 always @(negedge nclk)begin
   if(enable)begin
     if(!was && gap<12)$fatal(1,"IFG too short %d",gap);
     $fwrite(out,"%02x",gmii);cnt=cnt+1;gap=0;
   end else begin
     gap=gap+1;
     if(was)begin $fwrite(out,"\n");packets=packets+1;cnt=0;end
   end
   was=enable;
 end
 initial begin
 #200;rst=1;
 for(f=1;f<=3;f=f+1)begin
 @(negedge pclk);vs=1;repeat(4)@(negedge pclk);vs=0;
 for(y=0;y<4;y=y+1)begin
   for(x=0;x<640;x=x+1)begin
     @(negedge pclk);de=1;rgb={5'(x%32),3'b000,6'(y+f),2'b00,5'((x+y)%32),3'b000};
   end
   @(negedge pclk);de=0;repeat(160)@(negedge pclk);
 end
 repeat(100)@(negedge pclk);
 end
 #30000;
 if(packets!=12 || drop)$fatal(1,"Wrong count %d drop %b",packets,drop);
 $fclose(out);$display("PASS: 12 GMII packets, async clocks, IFG >=12");$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
