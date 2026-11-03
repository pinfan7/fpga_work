`timescale 1ns/1ps
module yellow_overlay_tb;
    reg clk=0;always #20 clk=~clk;
    reg rst_n=0,hs=0,vs=0,de=0;reg [23:0] rgb=24'h123456;
    reg valid=1;wire [23:0] out;wire oh,ov,od;integer x,y;
    yellow_box_overlay dut(.clk(clk),.rst_n(rst_n),.rgb_in(rgb),.hs_in(hs),.vs_in(vs),.de_in(de),
        .box_valid(valid),.x0(12'd20),.y0(12'd64),.x1(12'd59),.y1(12'd103),
        .rgb_out(out),.hs_out(oh),.vs_out(ov),.de_out(od));
    task check(input [23:0] expected);begin
        @(posedge clk);#1;
        if(out!==expected || oh!==hs || ov!==vs || od!==de)$fatal(1,"overlay RGB/sync mismatch x=%d y=%d out=%h",x,y,out);
        @(negedge clk);
    end endtask
    reg edgep,crossp;reg [23:0] expected;
    initial begin
        repeat(3)@(negedge clk);rst_n=1;vs=1;de=0;check(0);vs=0;check(0);
        for(y=0;y<128;y=y+1)begin
            for(x=0;x<160;x=x+1)begin
                de=1;hs=x[0];
                edgep=x>=20 && x<=59 && y>=64 && y<=103 && (x<22 || x>57 || y<66 || y>101);
                crossp=(x==39 && y>=78 && y<=88)||(y==83 && x>=34 && x<=44);
                expected=(edgep || crossp)?24'h00ff00:rgb;check(expected);
            end
            de=0;check(0);check(0);
        end
        $display("PASS yellow_overlay_tb: every RGB pixel and HS/VS/DE aligned");$finish;
    end
endmodule
